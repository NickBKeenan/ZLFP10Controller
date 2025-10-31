#include <EEPROM.h>



#include <ZLFP10Controller.h>
#include <LEDStatusStrip.h>

#define ERROR_CANT_CALIBRATE 1

#define WARNING_READ_HOLDING 1
#define WARNING_READ_INPUT 2
#define WARNING_DEFLUTTER 3
#define WARNING_DEFLUTTER_FAIL 4


void ZLFP10Controller::setRoomTempPin(uint8_t pRoomTempPin)
{
    RoomTempPin = pRoomTempPin;

    pinMode(RoomTempPin, OUTPUT);
    analogWrite(RoomTempPin, 134); // write a random value to silence the alarm
}
void ZLFP10Controller::setCoilTempPin(uint8_t pCoilTempPin)
{
    CoilTempPin = pCoilTempPin;

    pinMode(CoilTempPin, INPUT);

}

void ZLFP10Controller::setClientHardwareSerial(HardwareSerial& phwSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID)
{
    phwSerial.begin(9600);

    client.begin(ModbusID, phwSerial);

    client.SetPins(pRS485DEPin, pRS485REPin);



}

void ZLFP10Controller::setClientSoftwareSerial(SoftwareSerial& pswSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID)
{
    pswSerial.begin(9600);

    client.begin(ModbusID, pswSerial);

    client.SetPins(pRS485DEPin, pRS485REPin);



}

void ZLFP10Controller::setServerHardwareSerial(HardwareSerial& phwSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID, ModbusServer* p_pServer)
{
    phwSerial.begin(9600);

    pserver = p_pServer;

    pserver->begin(ModbusID, phwSerial);

    pserver->SetPins(pRS485DEPin, pRS485REPin);



}

void ZLFP10Controller::setServerSoftwareSerial(SoftwareSerial& pswSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID, ModbusServer* p_pServer)
{

    pswSerial.begin(9600);

    pserver = p_pServer;

    pserver->begin(ModbusID, pswSerial);

    pserver->SetPins(pRS485DEPin, pRS485REPin);



}

void ZLFP10Controller::ReadSettings() {


#define HOLDINGCOUNT  21
#define INPUTCOUNT  10
#define HOLDINGFIRST  0x6E8D
#define INPUTFIRST  0xB6D1

    short holdingData[HOLDINGCOUNT];
    short inputData[INPUTCOUNT];
    uint8_t  result;



    result = client.readHoldingRegisters(HOLDINGFIRST, HOLDINGCOUNT);
    if (result != 0)
    {
        theLEDStatusStrip.Warning(WARNING_READ_HOLDING);

        result = client.readHoldingRegisters(HOLDINGFIRST, HOLDINGCOUNT);
    }
    if (result != 0)
    {
        DebugStream->println("Error reading holding registers");
        theLEDStatusStrip.Warning(WARNING_READ_HOLDING);
        return;
    }
    int x;
    for (x = 0; x < HOLDINGCOUNT; x++) {
        holdingData[x] = client.ResponseBufferGetAt(x);

        /*              DebugStream->println();
                      DebugStream->print("Holdingdata ");
                      DebugStream->print(x);
                      DebugStream->print(" = ");
                      DebugStream->print(holdingData[x]);*/

    }
    delay(150);  // need a little bit of time between requests

    result = client.readInputRegisters(INPUTFIRST, INPUTCOUNT);
    if (result != 0)
    {
        theLEDStatusStrip.Warning(WARNING_READ_INPUT);
        result = client.readInputRegisters(INPUTFIRST, INPUTCOUNT);
    }
    if (result != 0)
    {
        DebugStream->println("Error reading Input registers");
        theLEDStatusStrip.Warning(WARNING_READ_INPUT);
        return;
    }

    for (x = 0; x < INPUTCOUNT; x++) {
        inputData[x] = client.ResponseBufferGetAt(x);
    }


    FCUSettings.Onoff = holdingData[0];
    FCUSettings.Mode = holdingData[1];
    FCUSettings.FanModeSetting = holdingData[2];
    FCUSettings.AutoCoolingSetpoint = holdingData[11] / 10;
    FCUSettings.AutoHeatingSetpoint = holdingData[12] / 10;
    FCUSettings.HeatingWaterMinTemp = holdingData[13] / 10;
    FCUSettings.HeatSetpoint = holdingData[10] / 10;
    FCUSettings.CoolSetpoint = holdingData[9] / 10;
    FCUSettings.RoomTemp = inputData[0] / 10;
    FCUSettings.coilTemp = inputData[1] / 10;
    FCUSettings.FanSetting = inputData[2];
    FCUSettings.FanRPM = inputData[3];
    FCUSettings.valveOpen = inputData[4];
    FCUSettings.FanFault = inputData[7];



    // fluttering detected, try to adjust setpoints
    if (lastTempSetting != 0 && FCUSettings.RoomTemp != lastTempSetting)// && millis()> nextAdjustmentTime && nextAdjustmentTime > 0)
    {
        DeFlutter();
        SetFanSpeed(lastSpeedSetting);

    }

}

void ZLFP10Controller::SetHoldingRegister(short RegisterID, short newValue)
{
    uint8_t retval;
    client.TransmitBufferPutAt(0, newValue);
    retval = client.writeMultipleRegisters(HOLDINGFIRST + RegisterID, 1);

}
// turn on and off
void ZLFP10Controller::SetOnOff(short isOn)
{

    SetHoldingRegister(0, isOn);
    FCUSettings.Onoff = isOn;

}
void ZLFP10Controller::SetFanModeSetting(short newSetting)
{
    SetHoldingRegister(2, newSetting);

}
void ZLFP10Controller::SetMode(short newMode)
{
    SetHoldingRegister(1, newMode);
    FCUSettings.Mode = newMode;
}
void ZLFP10Controller::SetSetpoint(short newSetpoint)
{
    if (Cooling())
    {
        SetHoldingRegister(9, newSetpoint * 10);
        FCUSettings.CoolSetpoint = newSetpoint;
    }

    else
    {
        SetHoldingRegister(10, newSetpoint * 10);
        FCUSettings.HeatSetpoint = newSetpoint;
    }
    // not sure if this is meaningful when mode is AUTO
}


void ZLFP10Controller::SetFanSpeed(int fanspeed)
{

    int newSetting;
    newSetting = SetLevels[fanspeed];
    analogWrite(RoomTempPin, newSetting);
    delay(1000); // to prevent read errors
    lastTempSetting = 0;

    if (Cooling())
    {
        if (fanspeed == 1)
        {
            pinMode(CoilTempPin, OUTPUT);
            digitalWrite(CoilTempPin, 0);
            SetMode(FCU_MODE_AUTO);
            delay(1000); // give it a seccond to catch up
        }
        else
        {

            pinMode(CoilTempPin, INPUT);
            if (FCUSettings.Mode == FCU_MODE_AUTO)
            {
                SetMode(FCU_MODE_COOL);
                delay(1000);
            }
        }
    }
    else
    {
        pinMode(CoilTempPin, INPUT);
    }




    if (Cooling())
    {
        if (fanspeed != 1)
        {

            lastTempSetting = FCUSettings.CoolSetpoint + fanspeed - 1;
        }
        else
        {
            lastTempSetting = FCUSettings.HeatSetpoint - fanspeed;
        }
    }
    else
    {

        lastTempSetting = FCUSettings.HeatSetpoint - fanspeed;
    }

    lastSpeedSetting = fanspeed;
    lastTempPin = newSetting;
    nextAdjustmentTime = millis() + (unsigned long)ADJUSTMENT_INTERVAL * 1000;
}

void ZLFP10Controller::GetCalibrationInfo()
{


    // as my tribute to Mark Zbikowski, we mark the first two bytes of the EEPROM with "NK" to signify that they've been written to
    byte N = EEPROM[0];
    byte K = EEPROM[1];
    if (N == 'N' && K == 'K')  // array is initialized
    {
        Serial.println();
        Serial.print("Loading from EEPROM");

        int x;
        for (x = MinTemp; x <= MaxTemp; x++)
        {
            CalibrationInfo[x - MinTemp] = EEPROM[x - MinTemp + 2];
            Serial.println();
            Serial.print(x);
            Serial.print(" ");
            Serial.print(CalibrationInfo[x - MinTemp]);

        }

    }
    else
    {
        int x;
        ReadSettings();

        SetOnOff(0); // ignore the temperature until we're done calibrating
        short temp = MinTemp;
        short prevtemp = 0;
        short startx = 0;

        for (x = 170; x < 255 && temp >= MinTemp; x++)
        {
            analogWrite(RoomTempPin, x);
            delay(250);
            ReadSettings();

            temp = FCUSettings.RoomTemp;
            Serial.print(".");
        }


        for (; x > 0 && temp < 31; x--)
        {
            analogWrite(RoomTempPin, x);
            delay(1000);
            ReadSettings();
            temp = FCUSettings.RoomTemp;
            if (prevtemp == 0)
                prevtemp = temp;
            if (temp != prevtemp)
            {
                if (startx != 0)
                {
                    Serial.println();
                    Serial.print("end=");
                    Serial.print(x + 1);
                    Serial.print(" start =");
                    Serial.print(startx);
                    Serial.print(" Temp=");
                    Serial.print(prevtemp);
                    Serial.print(" setting =");
                    Serial.print((startx + x + 1) / 2);
                    if (prevtemp >= MinTemp && prevtemp <= MaxTemp)
                    {
                        CalibrationInfo[prevtemp - MinTemp] = (startx + x + 1) / 2;
                    }
                }
                prevtemp = temp;
                startx = x;

            }

        }

        for (x = MinTemp; x < MaxTemp + 1; x++)
        {
            Serial.println("");
            if (CalibrationInfo[x - MinTemp] == 0)
            {
                if (x > MinTemp && x < MaxTemp)
                {
                    Serial.print("*");
                    CalibrationInfo[x - MinTemp] = (CalibrationInfo[x - MinTemp - 1] + CalibrationInfo[x - MinTemp + 1]) / 2;
                }
            }

            Serial.print(x);
            Serial.print(" ");
            Serial.print(CalibrationInfo[x - MinTemp]);
            EEPROM[x - MinTemp + 2] = CalibrationInfo[x - MinTemp];
        }
        EEPROM[0] = 'N';
        EEPROM[1] = 'K';
        Serial.println("Done");



    }


}

void ZLFP10Controller::AdjustCalibrationData()
{
    // called after defluttering -- our temperture map was wrong and has been corrected

    // the entry for lastTempSetting needs to be changed to what's in  SetLevels[lastSpeedSetting]
    CalibrationInfo[lastTempSetting - MinTemp] = SetLevels[lastSpeedSetting];
    EEPROM[lastTempSetting - MinTemp + 2] = SetLevels[lastSpeedSetting];
}

void ZLFP10Controller::Calibrate()
{
    DebugStream->println("Calibrating...");
    ReadSettings();
    int CoolSetpoint = FCUSettings.CoolSetpoint;
    int HeatSetpoint = FCUSettings.HeatSetpoint;

    if (CoolSetpoint > MaxTemp - 3)
        CoolSetpoint = MaxTemp - 3;
    if (CoolSetpoint < MinTemp + 1)
        CoolSetpoint = MinTemp + 1;
    if (HeatSetpoint > MaxTemp)
        HeatSetpoint = MaxTemp;
    if (HeatSetpoint < MinTemp + 4)
        HeatSetpoint = MinTemp + 4;

    CoolSetpoint -= MinTemp;
    HeatSetpoint -= MinTemp;

    if (Cooling())
    {
        SetLevels[0] = CalibrationInfo[CoolSetpoint - 1];
        SetLevels[1] = CalibrationInfo[(HeatSetpoint - 1)]; // speed 1 is going to be auto mode, heating spoofed
        SetLevels[2] = CalibrationInfo[(CoolSetpoint + 1)];
        SetLevels[3] = CalibrationInfo[(CoolSetpoint + 2)];
        SetLevels[4] = CalibrationInfo[(CoolSetpoint + 3)];

    }
    else
    {
        SetLevels[0] = CalibrationInfo[HeatSetpoint];
        SetLevels[1] = CalibrationInfo[(HeatSetpoint - 1)];
        SetLevels[2] = CalibrationInfo[(HeatSetpoint - 2)];
        SetLevels[3] = CalibrationInfo[(HeatSetpoint - 3)];
        SetLevels[4] = CalibrationInfo[(HeatSetpoint - 4)];
    }

    int x;
    for (x = 0; x < 5; x++)
    {
        DebugStream->println(SetLevels[x]);
    }
    lastTempSetting = 0; // have to do this to prevent defluttering


    DebugStream->println("Done Calibrating");


}


void ZLFP10Controller::DeFlutter()
{
    // we get here when we set the temperature, and then we read it back and it's not what we set it to.
    // we have:
    // settemp -- thermostat setting on the unit
    // reportedRoomTemp  -- what the unit is reporting
    // lastTempSetting -- what we set the temp to
    DebugStream->println();
    DebugStream->print("DEFluttering. Settemp=");
    DebugStream->print(lastTempSetting);
    DebugStream->print(" reported temp=");
    DebugStream->print(FCUSettings.RoomTemp);
    DebugStream->println();

    theLEDStatusStrip.Warning(WARNING_DEFLUTTER);

    if (Cooling())
    {
        if (lastSpeedSetting == MINFANSPEED || lastSpeedSetting == 1)  // zero
        {
            if (FCUSettings.RoomTemp > lastTempSetting)
            {
                SetLevels[lastSpeedSetting]++;
            }
            else
            {
                SetLevels[lastSpeedSetting]--;
            }
            DebugStream->print("Adjusting level ");
            DebugStream->print(lastSpeedSetting);

            DebugStream->print(SetLevels[lastSpeedSetting]);
            DebugStream->println();
            return;

        }
        if (lastSpeedSetting == MAXFANSPEED)
        {
            if (FCUSettings.RoomTemp < lastTempSetting)
            {
                SetLevels[MAXFANSPEED]--;
                DebugStream->print("Adjusting level ");
                DebugStream->print(MAXFANSPEED);
                DebugStream->print(" to ");
                DebugStream->print(SetLevels[MAXFANSPEED]);
                DebugStream->println();
                return;
            }
        }
        if (FCUSettings.RoomTemp < lastTempSetting)
        {
            // 
            int setting = lastSpeedSetting;
            DebugStream->print(" Setting=");
            DebugStream->print(setting);
            DebugStream->print(" Level=");
            DebugStream->print(SetLevels[setting]);
            DebugStream->print(" Next Level=");
            DebugStream->print(SetLevels[setting + 1]);

            if (SetLevels[setting] - SetLevels[setting + 1] > 1)
            {
                SetLevels[setting]--;
                DebugStream->print("Adjusting level ");
                DebugStream->print(setting);
                DebugStream->print(" to ");
                DebugStream->print(SetLevels[setting]);
                DebugStream->println();
            }
            else
            {
                DebugStream->println("Unable to deflutter.");
                theLEDStatusStrip.Warning(WARNING_DEFLUTTER_FAIL);
            }
        }
        else // reported temp is > than setting
        {

            int setting = lastSpeedSetting;
            if (SetLevels[setting - 1] - SetLevels[setting] > 1)
            {
                SetLevels[setting]++;
                DebugStream->print("Adjusting level ");
                DebugStream->print(setting);
                DebugStream->print(" to ");
                DebugStream->print(SetLevels[setting]);
            }
            else
            {
                DebugStream->println("Unable to deflutter.");
                theLEDStatusStrip.Warning(WARNING_DEFLUTTER_FAIL);
            }

        }
    }
    else
    {
        if (lastSpeedSetting == MINFANSPEED)
        {
            if (FCUSettings.RoomTemp < lastTempSetting)
            {
                SetLevels[MINFANSPEED]--;
                DebugStream->print("Adjusting level 0 to ");
                DebugStream->print(SetLevels[MINFANSPEED]);
                DebugStream->println();
                return;
            }
        }
        if (lastSpeedSetting == MAXFANSPEED)
        {
            if (FCUSettings.RoomTemp > lastTempSetting)
            {
                SetLevels[MAXFANSPEED]++;
                DebugStream->print("Adjusting level ");
                DebugStream->print(MAXFANSPEED);
                DebugStream->print(" to ");
                DebugStream->print(SetLevels[MAXFANSPEED]);
                DebugStream->println();
                return;
            }
        }
        if (FCUSettings.RoomTemp > lastTempSetting)
        {
            // 
            int setting = lastSpeedSetting;
            DebugStream->print(" Setting=");
            DebugStream->print(setting);
            DebugStream->print(" Level=");
            DebugStream->print(SetLevels[setting]);
            DebugStream->print(" Next Level=");
            DebugStream->print(SetLevels[setting + 1]);

            if (SetLevels[setting + 1] - SetLevels[setting] > 1)
            {
                SetLevels[setting]++;
                DebugStream->print("Adjusting level ");
                DebugStream->print(setting);
                DebugStream->print(" to ");
                DebugStream->print(SetLevels[setting]);
                DebugStream->println();
            }
            else
            {
                DebugStream->println("Unable to deflutter.");
            }
        }
        else
        {

            int setting = lastSpeedSetting;
            if (SetLevels[setting] - SetLevels[setting - 1] > 1)
            {
                SetLevels[setting]--;
                DebugStream->print("Adjusting level ");
                DebugStream->print(setting);
                DebugStream->print(" to ");
                DebugStream->print(SetLevels[setting]);
            }
            else
            {
                DebugStream->println("Unable to deflutter.");
            }

        }
    }



}
void ZLFP10Controller::SetDebugOutput(Stream* pDebug)
{
    DebugStream = pDebug;
}

bool ZLFP10Controller::Cooling()
{
    if (FCUSettings.Mode == FCU_MODE_COOL || FCUSettings.Mode == FCU_MODE_AUTO)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void ZLFP10Controller::ServiceAnyRequests()
{
    pserver->ServiceAnyRequests();



}

ModbusClient* ZLFP10Controller::GetClient()
{
    return &client;
}