


#include <ZLFP10Controller.h>
#include <LEDStatusStrip.h>

#define ERROR_CANT_CALIBRATE 1

#define WARNING_READ_HOLDING 1
#define WARNING_READ_INPUT 2
#define WARNING_DEFLUTTER 3
#define WARNING_DEFLUTTER_FAIL 4


void ZLFP10Controller::setRoomTempPin(uint8_t pRoomTempPin)
{
  RoomTempPin= pRoomTempPin;

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

    client.begin( ModbusID, phwSerial);

    client.SetPins(pRS485DEPin, pRS485REPin);



}

void ZLFP10Controller::setClientSoftwareSerial(SoftwareSerial &pswSerial, uint8_t pRS485DEPin,uint8_t pRS485REPin, uint8_t ModbusID)
{
    pswSerial.begin(9600);

    client.begin(ModbusID, pswSerial);

  client.SetPins(pRS485DEPin, pRS485REPin);



}

void ZLFP10Controller::setServerHardwareSerial(HardwareSerial& phwSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID, ModbusServer * p_pServer)
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

    pserver->begin( ModbusID, pswSerial);

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
    

     
    result = client.readHoldingRegisters( HOLDINGFIRST, HOLDINGCOUNT);
    if (result != 0)
    {
        theLEDStatusStrip.Warning(WARNING_READ_HOLDING);
        
        result = client.readHoldingRegisters(HOLDINGFIRST, HOLDINGCOUNT);
    }
    if(result !=0)
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

    result = client.readInputRegisters( INPUTFIRST, INPUTCOUNT);
    if (result != 0)
    {
        theLEDStatusStrip.Warning(WARNING_READ_INPUT);
        result = client.readInputRegisters(INPUTFIRST, INPUTCOUNT);
    }
    if(result !=0)
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
    FCUSettings.FanModeSetting= holdingData[2];
    FCUSettings.AutoCoolingSetpoint = holdingData[11] / 10;
    FCUSettings.AutoHeatingSetpoint = holdingData[12] / 10;
    FCUSettings.HeatingWaterMinTemp= holdingData[13] / 10;
    FCUSettings.HeatSetpoint = holdingData[10] / 10;
    FCUSettings.CoolSetpoint= holdingData[9] / 10;
    FCUSettings.RoomTemp = inputData[0] / 10;
    FCUSettings.coilTemp = inputData[1] / 10;
    FCUSettings.FanSetting = inputData[2];
    FCUSettings.FanRPM= inputData[3];
    FCUSettings.valveOpen = inputData[4];
    FCUSettings.FanFault = inputData[7];



    // fluttering detected, try to adjust setpoints
    if(lastTempSetting !=0 && FCUSettings.RoomTemp != lastTempSetting)// && millis()> nextAdjustmentTime && nextAdjustmentTime > 0)
    {
        DeFlutter();
        SetFanSpeed(lastSpeedSetting);

    }

}

void ZLFP10Controller::SetHoldingRegister(short RegisterID, short newValue)
{
    uint8_t retval;
    client.TransmitBufferPutAt(0, newValue);
    retval = client.writeMultipleRegisters(HOLDINGFIRST+RegisterID, 1);

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
        SetHoldingRegister(10, newSetpoint*10);
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

    if (Cooling() )
    {
        if( fanspeed == 1)
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
      
        lastTempSetting = FCUSettings.CoolSetpoint + fanspeed-1;
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
  nextAdjustmentTime = millis() + (unsigned long)ADJUSTMENT_INTERVAL*1000;
}


void ZLFP10Controller::Calibrate()
{
    DebugStream->println("Calibrating...");

// the fan speed is setpoint minus reported temperature
  // Using the onboard 5V, it seems that reported is roughly 165-input
  
  // So estimate the extremes of our range, 15C and 30C, measure what we get, and then interpolate the points in between
  // Then test them against actual and adjust

  // We only need three points 1,2,3. Zero is anything less than one, and four is anything less than three. 
  //So figure out those three and then substract two from one to get zero, and add two to three to get four
  
    lastTempSetting = 0; // have to do this to prevent defluttering
  short LevelL, LevelH;   // input levels for the extremes, low and high
  short ReadingL, ReadingH; // room temperature readings we get back
  LevelL= 135; // set for 15 C
  LevelH=120;  // set for 30 C

  int iterator = 0;
  {/*
      int x;
      int lastread = 99;
      for (x = 105; x < 255&& lastread > 9; x+=1)
      {
          analogWrite(RoomTempPin, x);
          delay(500);
          ReadSettings();
          Serial.println();
          Serial.print("setting=");
          Serial.print(x);
          Serial.print(" reading=");
          Serial.print(FCUSettings.RoomTemp);
          lastread = FCUSettings.RoomTemp;
          Serial.print(" Fanspeed=");
          Serial.print(FCUSettings.FanSetting);
          Serial.print(" FanRPM=");
          Serial.print(FCUSettings.FanRPM);
      }
      while (1);/*
      analogWrite(RoomTempPin, 1);
      delay(1000);
      ReadSettings();
      Serial.println();
      Serial.print("setting=");
      Serial.print(1);
      Serial.print(" reading=");
      Serial.print(FCUSettings.RoomTemp);
      delay(10000);
      analogWrite(RoomTempPin, 254);
      delay(1000);
      ReadSettings();
      Serial.println();
      Serial.print("setting=");
      Serial.print(254);
      Serial.print(" reading=");
      Serial.print(FCUSettings.RoomTemp);

      while (1);
      //*/
  }

  //check out low end
  theLEDStatusStrip.SetStatus((iterator++ % 4) + 1);
  analogWrite(RoomTempPin, LevelL);
  delay(1000);
  ReadSettings();
  ReadingL=FCUSettings.RoomTemp;

  // check out high end
  theLEDStatusStrip.SetStatus((iterator++ % 4) + 1);
  analogWrite(RoomTempPin, LevelH);
  delay(1000);
  ReadSettings();
  ReadingH= FCUSettings.RoomTemp;
  DebugStream->print(LevelL); //150
  DebugStream->print(" ");
  DebugStream->print(ReadingL); //170 17c
  DebugStream->print(" ");
  DebugStream->print(LevelH); //135
  DebugStream->print(" ");
  DebugStream->print(ReadingH); //300 30c


  // (LevelH-TargetLevel)/(LevelH-LevlL)= (ReadingH-TargetReading)/(ReadingH-ReadingL); or
  // TargetLevel= LevelH - (LevelH-LevelL)*(ReadingH-TargetReading)/(ReadingH-ReadingL)
// get initial guesses based on what the extremes of the range told us

  
  
  


  DebugStream->println();
  short temps[4];

  if (Cooling())
  {

      
      DebugStream->print("Mode is cooling");
      DebugStream->print(" Setpoint is:");

      DebugStream->print(FCUSettings.CoolSetpoint);

      // set autoheat setpoint
      SetHoldingRegister(12, FCUSettings.HeatSetpoint*10);
      temps[0] = FCUSettings.CoolSetpoint-1;
      temps[1] = (FCUSettings.HeatSetpoint - 1); // speed 1 is going to be auto mode, heating spoofed
      temps[2] = (FCUSettings.CoolSetpoint + 1);
      temps[3] = (FCUSettings.CoolSetpoint + 2);

  }
  else
  {
      DebugStream->print("Mode is heating");
      DebugStream->print(" Setpoint is:");

      DebugStream->print(FCUSettings.HeatSetpoint);
      temps[0] = FCUSettings.HeatSetpoint;
      temps[1] = (FCUSettings.HeatSetpoint - 1);
      temps[2] = (FCUSettings.HeatSetpoint - 2);
      temps[3] = (FCUSettings.HeatSetpoint - 3);
  }

  


  

  
  
  SetLevels[0] = LevelH + (LevelL - LevelH) * (ReadingH - temps[0]) / (ReadingH - ReadingL);
  SetLevels[1]=LevelH + (LevelL-LevelH)*(ReadingH-temps[1])/(ReadingH-ReadingL);
  SetLevels[2]=LevelH + (LevelL-LevelH)*(ReadingH-temps[2])/(ReadingH-ReadingL);
  SetLevels[3]=LevelH + (LevelL-LevelH)*(ReadingH-temps[3])/(ReadingH-ReadingL);

  int x;
  
  // go through the settings, set each one and then compare what happens and adjust
  // Keep going until you can do it three times without adjustment, or 30 times total (failure)
 int goodTries=0; // number of sucessful reads   
  for(x=0; x< 30 && goodTries < 3; x++)
  {
    int y;
    DebugStream->println();
    int goodLevels=0;    // number of successful reads in this iteration
    for(y=0; y< 4; y++)
    {
      short reading;
      
      DebugStream->print(SetLevels[y]);
      analogWrite(RoomTempPin, SetLevels[y]);
      delay(1000);
      ReadSettings();
      reading=FCUSettings.RoomTemp;
      DebugStream->print(" ");
      DebugStream->print(reading);
      DebugStream->print(" . ");
      theLEDStatusStrip.SetStatus((iterator++ % 4)+1);

      if(reading != temps[y] )
      {// need adjustment
        DebugStream->print(" * ");
        SetLevels[y] += (reading- temps[y ]);
        DebugStream->print(SetLevels[y]);
        DebugStream->print(" * ");
        
      }
      else
      {// it works!
        DebugStream->print(" - ");
        goodLevels++;
      }
      
    }// go through each level
    if(goodLevels==4) // every read on this try was successful
    {
      goodTries++;
    }
    else
    {
      goodTries=0; // reset to zero on failure, it has to be three in a row because failure also mean we adjust the values
    }
  
  }// tries

  if(Cooling())
  {

      SetLevels[MAXFANSPEED] = SetLevels[3] - 2;
  }
  else
  {
    
    SetLevels[MAXFANSPEED]=SetLevels[3]+2;
  }
  
  
  if(goodTries==3)
  {
    DebugStream->println("Done Calibrating");
    
  }
  else
  {
      DebugStream->println("Unable to calibrate");
    // trigger a P5 error
    analogWrite(RoomTempPin,0);
    theLEDStatusStrip.FatalError(ERROR_CANT_CALIBRATE);

    while(1);
  }
   
  

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

  if(Cooling())
  {
      if (lastSpeedSetting == MINFANSPEED || lastSpeedSetting== 1)  // zero
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
    if(lastSpeedSetting == MAXFANSPEED)
    {
        if(FCUSettings.RoomTemp> lastTempSetting )
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
    if(FCUSettings.RoomTemp > lastTempSetting)
    {
      // 
      int setting =lastSpeedSetting;
      DebugStream->print(" Setting=");
      DebugStream->print(setting);
      DebugStream->print(" Level=");
      DebugStream->print(SetLevels[setting]);
      DebugStream->print(" Next Level=");
      DebugStream->print(SetLevels[setting+1]);

      if(SetLevels[setting+1]-SetLevels[setting]>1)
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
      if(SetLevels[setting]-SetLevels[setting-1]>1)
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