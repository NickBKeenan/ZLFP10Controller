#ifndef ZLFP10Controller_h
#define ZLFP10Controller_h

#include <ModbusMaster.h>
#include <SoftwareSerial.h>

// Set up a new SoftwareSerial object
#define MAXFANSPEED 4
#define MINFANSPEED 0  //  0=off

#define ADJUSTMENT_INTERVAL 180 // don't adust more than once every 3 minutes
#define UNIT_ADDRESS 15 // the MODBUS address of the FCU. This can be changed through the control panel if needed.
#define FCU_MODE_AUTO 0
#define FCU_MODE_COOL 1
#define FCU_MODE_DEHU 2
#define FCU_MODE_VENT 3
#define FCU_MODE_HEAT 4

struct settingsHolder {
    short Onoff;
    short Mode ;
    short AutoCoolingSetpoint;
    short AutoHeatingSetpoint;
    short HeatSetpoint ;
    short CoolSetpoint ;
    short RoomTemp ;
    short coilTemp ;
    short FanSetting ;
    short FanRPM ;
    short valveOpen;

};

class ZLFP10Controller
{

    ModbusMaster node;

    int TempReaderPin; // PWm pin that connects to FCU temp reader
    unsigned long nextAdjustmentTime = 0; //don't deflutter more than once every three minutes

  
public:
    settingsHolder FCUSettings;
    int lastTempPin;
private:
    short lastTempSetting=0; // used for deflutter
    int lastSpeedSetting = 0; // used for deflutter
    bool isHeat; //used for deflutter

    short SetLevels[5]; // the digital output level for each fan speed, set in Calibrate(); 
    Stream *DebugStream;

public:
    
    void setSerial(SoftwareSerial &pswSerial, uint8_t pRS485DEPin,uint8_t pRS485REPin);
    void setTempReader(uint8_t pTempReaderPin);
    void setup();
    void ReadSettings(); // read from onboard settings
    void SetFanSpeed(int fanspeed,  int targettemp); // write out to temperature spoof
    void Calibrate(bool isheating, int FCUSetTemp);   // calibrate temperature spoofing
    void DeFlutter();  // if temperature fluttering is detected, revisit the calibration
    void SetDebugOutput(Stream* pDebug);

};

#endif