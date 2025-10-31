//
#ifndef LEDStatusStrip_h
#define LEDStatusStrip_h


class LEDStatusStrip
{
    int LEDCount=0;
    int BasePin;
    int oldStatus;
public:
    static bool ledsEnabled;
    static void setLEDsEnabled(bool enabled) { ledsEnabled = enabled; }
    void SetStatus(int newStatus);
    void Warning(int WarningLevel);
    void FatalError(int ErrorLevel);
    void SetPins(int firstpin, int pinCount);
    void BlinkEm(int number, unsigned long duration);
};
extern LEDStatusStrip theLEDStatusStrip;
#endif
