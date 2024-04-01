


#include <ZLFP10Controller.h>

#define ERROR_CANT_CALIBRATE 1

#define WARNING_READ_HOLDING 1
#define WARNING_READ_INPUT 2


void ZLFP10Controller::setTempReader(uint8_t pTempReaderPin)
{
  TempReaderPin= pTempReaderPin;

  pinMode(TempReaderPin, OUTPUT);
  analogWrite(TempReaderPin, 134); // write a random value to silence the alarm
}
void ZLFP10Controller::setSerial(SoftwareSerial &pswSerial, uint8_t pRS485DEPin,uint8_t pRS485REPin)
{
    pswSerial.begin(9600);

    node.begin(15, pswSerial);

  node.SetPins(pRS485DEPin, pRS485REPin);



}

void ZLFP10Controller::setup() 
{


    
  
}

void ZLFP10Controller::ReadSettings() {
  
    
    #define HOLDINGCOUNT  21
    #define INPUTCOUNT  10
    #define HOLDINGFIRST  0x6E8D
    #define INPUTFIRST  0xB6D1

    short holdingData[HOLDINGCOUNT];
    short inputData[INPUTCOUNT];
      uint8_t  result;
    

     
    result = node.readHoldingRegisters( HOLDINGFIRST, HOLDINGCOUNT);
    if (result != 0)
    {
        Warning(WARNING_READ_HOLDING);
        
        result = node.readHoldingRegisters(HOLDINGFIRST, HOLDINGCOUNT);
    }
    if(result !=0)
    {
        DebugStream->println("Error reading holding registers");
        Warning(WARNING_READ_HOLDING);
      return;
    }
    int x;
    for (x = 0; x < HOLDINGCOUNT; x++) {
              holdingData[x] = node.getResponseBuffer(x);


    }
    delay(150);  // need a little bit of time between requests

    result = node.readInputRegisters( INPUTFIRST, INPUTCOUNT);
    if (result != 0)
    {
        Warning(WARNING_READ_INPUT);
        result = node.readInputRegisters(INPUTFIRST, INPUTCOUNT);
    }
    if(result !=0)
    {
        DebugStream->println("Error reading Input registers");
        Warning(WARNING_READ_INPUT);
      return;
    }

    for (x = 0; x < INPUTCOUNT; x++) {
        inputData[x] = node.getResponseBuffer(x);
    }

    
    FCUSettings.Onoff = holdingData[0];
    FCUSettings.Mode = holdingData[1];
    FCUSettings.AutoCoolingSetpoint = holdingData[11] / 10;
    FCUSettings.AutoHeatingSetpoint = holdingData[12] / 10;
    FCUSettings.HeatSetpoint = holdingData[10] / 10;
    FCUSettings.CoolSetpoint= holdingData[9] / 10;
    FCUSettings.RoomTemp = inputData[0] / 10;
    FCUSettings.coilTemp = inputData[1] / 10;
    FCUSettings.FanSetting = inputData[2];
    FCUSettings.FanRPM= inputData[3];
    FCUSettings.valveOpen = inputData[4];
    FCUSettings.FanFault = inputData[7];

// fan setting is holdingData[2];

    // fluttering detected, try to adjust setpoints
    if(lastTempSetting !=0 && FCUSettings.RoomTemp != lastTempSetting)// && millis()> nextAdjustmentTime && nextAdjustmentTime > 0)
    {
        DeFlutter();
        SetFanSpeed(lastSpeedSetting, lastTempSetting);

    }

}
  // turn on and off
void ZLFP10Controller::SetOnOff(short isOn)
{
    uint8_t retval;
    node.setTransmitBuffer(0, isOn);
    retval=node.writeMultipleRegisters(HOLDINGFIRST, 1);



}

void ZLFP10Controller::SetFanSpeed(int fanspeed, int targettemp)
{
  // version 3 -- look up setting in the table created in Calibrate()

  int newSetting;
  newSetting=SetLevels[fanspeed];
  analogWrite( TempReaderPin, newSetting); 
  delay(500); // to prevent read errors
  
  lastTempSetting=targettemp;
  lastSpeedSetting = fanspeed;
  lastTempPin = newSetting;
  nextAdjustmentTime = millis() + (unsigned long)ADJUSTMENT_INTERVAL*1000;
}


void ZLFP10Controller::Calibrate(bool isHeating, int FCUSetTemp)
{
    DebugStream->println("Calibrating...");
// the fan speed is setpoint minus reported temperature
  // Using the onboard 5V, it seems that reported is roughly 165-input
  
  // So estimate the extremes of our range, 15C and 30C, measure what we get, and then interpolate the points in between
  // Then test them against actual and adjust

  // We only need three points 1,2,3. Zero is anything less than one, and four is anything less than three. 
  //So figure out those three and then substract two from one to get zero, and add two to three to get four
  
    isHeat = isHeating;
    lastTempSetting = 0; // have to do this to prevent defluttering
  short LevelL, LevelH;   // input levels for the extremes, low and high
  short ReadingL, ReadingH; // room temperature readings we get back
  LevelL= 135; // set for 15 C
  LevelH=120;  // set for 30 C

  int iterator = 0;
  {/*
      int x;
      for (x = 1; x < 255; x+=1)
      {
          analogWrite(TempReaderPin, x);
          delay(1000);
          ReadSettings();
          Serial.println();
          Serial.print("setting=");
          Serial.print(x);
          Serial.print(" reading=");
          Serial.print(FCUSettings.RoomTemp);
      }
      while (1);
      analogWrite(TempReaderPin, 1);
      delay(1000);
      ReadSettings();
      Serial.println();
      Serial.print("setting=");
      Serial.print(1);
      Serial.print(" reading=");
      Serial.print(FCUSettings.RoomTemp);
      delay(10000);
      analogWrite(TempReaderPin, 254);
      delay(1000);
      ReadSettings();
      Serial.println();
      Serial.print("setting=");
      Serial.print(254);
      Serial.print(" reading=");
      Serial.print(FCUSettings.RoomTemp);

      while (1);
      */
  }

  //check out low end
  SetStatus((iterator++ % 4) + 1);
  analogWrite(TempReaderPin, LevelL);
  delay(1000);
  ReadSettings();
  ReadingL=FCUSettings.RoomTemp;

  // check out high end
  SetStatus((iterator++ % 4) + 1);
  analogWrite(TempReaderPin, LevelH);
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
  short temps[3];
  
  if(isHeating)
  {
    temps[0]=(FCUSetTemp-1);
    temps[1]=(FCUSetTemp-2);
    temps[2]=(FCUSetTemp-3);
  }
  else
  {
    temps[0]=(FCUSetTemp+1);
    temps[1]=(FCUSetTemp+2);
    temps[2]=(FCUSetTemp+3);

  }
  SetLevels[1]=LevelH + (LevelL-LevelH)*(ReadingH-temps[0])/(ReadingH-ReadingL);
  SetLevels[2]=LevelH + (LevelL-LevelH)*(ReadingH-temps[1])/(ReadingH-ReadingL);
  SetLevels[3]=LevelH + (LevelL-LevelH)*(ReadingH-temps[2])/(ReadingH-ReadingL);

// on the basement FCU, both 156 and 157 map to 20C. However, 156 flutters with 21C
// This line forces it to pick 157, which seems stable
// I'm not sure how to solve this, one idea would be to adaptively adjust the values when flutter happens.
 // SetLevels[2]=157;
  int x;
  
  // go through the settings, set each one and then compare what happens and adjust
  // Keep going until you can do it three times without adjustment, or 30 times total (failure)
 int goodTries=0; // number of sucessful reads   
  for(x=0; x< 30 && goodTries < 3; x++)
  {
    int y;
    DebugStream->println();
    int goodLevels=0;    // number of successful reads in this iteration
    for(y=1; y< 4; y++)
    {
      short reading;
      
      DebugStream->print(SetLevels[y]);
      analogWrite(TempReaderPin, SetLevels[y]);
      delay(1000);
      ReadSettings();
      reading=FCUSettings.RoomTemp;
      DebugStream->print(" ");
      DebugStream->print(reading);
      DebugStream->print(" . ");
      SetStatus((iterator++ % 4)+1);

      if(reading != temps[y-1] )
      {// need adjustment
        DebugStream->print(" * ");
        SetLevels[y] += (reading-FCUSetTemp+y);
        DebugStream->print(SetLevels[y]);
        DebugStream->print(" * ");
        
      }
      else
      {// it works!
        DebugStream->print(" - ");
        goodLevels++;
      }
      
    }// go through each level
    if(goodLevels==3) // every read on this try was successful
    {
      goodTries++;
    }
    else
    {
      goodTries=0; // reset to zero on failure, it has to be three in a row because failure also mean we adjust the values
    }
  
  }// tries

  if(isHeating)
  {
    SetLevels[MINFANSPEED]=SetLevels[1]-2;
    SetLevels[MAXFANSPEED]=SetLevels[3]+2;
  }
  else
  {
    SetLevels[MINFANSPEED]=SetLevels[1]+2;
    SetLevels[MAXFANSPEED]=SetLevels[3]-2;
  }
  if(goodTries==3)
  {
    DebugStream->println("Done Calibrating");
    
  }
  else
  {
      DebugStream->println("Unable to calibrate");
    // trigger a P5 error
    analogWrite(TempReaderPin,0);
    FatalError(ERROR_CANT_CALIBRATE);

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


  if(isHeat)
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
  else
  {
      if (lastSpeedSetting == MINFANSPEED)
      {
          if (FCUSettings.RoomTemp > lastTempSetting)
          {
              SetLevels[MINFANSPEED]++;
              DebugStream->print("Adjusting level 0 to ");
              DebugStream->print(SetLevels[MINFANSPEED]);
              DebugStream->println();
              return;
          }
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

          if (SetLevels[setting ] - SetLevels[setting+1] > 1)
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
          }
      }
      else
      {

          int setting = lastSpeedSetting;
          if (SetLevels[setting-1] - SetLevels[setting ] > 1)
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
          }

      }
  }

}
void ZLFP10Controller::SetDebugOutput(Stream* pDebug)
{
    DebugStream = pDebug;
}