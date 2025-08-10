#ifndef ZLFP10Controller_h
#define ZLFP10Controller_h

#include <ModbusClient.h>
#include <ModbusServer.h>
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
    short FanModeSetting;
    short FanRPM ;
    short valveOpen;
    short FanFault;
    short TempFault;
    short CoilTempFault;
    short HeatingWaterMinTemp;

};

class debugStream : public Stream
{
public:
    virtual void BlinkEm();
};


class ZLFP10Controller
{

    ModbusClient client;
    ModbusServer *pserver;

    int RoomTempPin; // PWm pin that connects to FCU room temp reader
    int CoilTempPin; //digital pin that connects to FCU coil temp 
    unsigned long nextAdjustmentTime = 0; //don't deflutter more than once every three minutes
    
  
public:
    settingsHolder FCUSettings;
    int lastTempPin;
private:
    short lastTempSetting=0; // used for deflutter
    int lastSpeedSetting = 0; // used for deflutter


    short SetLevels[5]; // the digital output level for each fan speed, set in Calibrate(); 
    Print *DebugStream;
    
public:
    // one of these two is used
    void setClientSoftwareSerial(SoftwareSerial &pswSerial, uint8_t pRS485DEPin,uint8_t pRS485REPin, uint8_t ModbusID);
    void setClientHardwareSerial(HardwareSerial& pswSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID);
    // one of these two is used
    void setServerSoftwareSerial(SoftwareSerial& pswSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID, ModbusServer* p_pServer);
    void setServerHardwareSerial(HardwareSerial& pswSerial, uint8_t pRS485DEPin, uint8_t pRS485REPin, uint8_t ModbusID, ModbusServer* p_pServer);

    void setRoomTempPin(uint8_t pRoomTempPin);
    void setCoilTempPin(uint8_t pCoilTempPin);  
    
    void ReadSettings(); // read from onboard settings
    void SetFanSpeed(int fanspeed); // write out to temperature spoof
    void Calibrate();   // calibrate temperature spoofing
    void DeFlutter();  // if temperature fluttering is detected, revisit the calibration
    void SetDebugOutput(Stream* pDebug);
    void SetOnOff(short isOn);  // turn on and off
    void SetMode(short newMode);
    void SetFanModeSetting(short newSetting);
    void SetSetpoint(short newSetpoint);
    void SetHoldingRegister(short RegisterID, short newValue);
    bool Cooling();
    void ServiceAnyRequests(); // called when a server needs something from the client
    ModbusClient* GetClient(); // returns a pointer to the client object
};

#endif