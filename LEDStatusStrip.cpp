#include <ZLFP10Controller.h>
#include "LEDStatusStrip.h"
#include <Arduino.h>

#define SHORT_BLINK 100
#define LONG_BLINK 500

LEDStatusStrip theLEDStatusStrip;
bool LEDStatusStrip::ledsEnabled = true;

void LEDStatusStrip::SetPins(int firstpin, int pinCount)
{
    if (!ledsEnabled) {
        LEDCount = 0;
        return;
    }
    LEDCount = pinCount;
    BasePin = firstpin;
    int x;
    for (x = 0; x < LEDCount; x++)
    {
        pinMode(BasePin + x, OUTPUT);
    }
}
void LEDStatusStrip::SetStatus(int newStatus)
{
    if (!ledsEnabled || LEDCount == 0)
    {
        return;
    }
    int x;
    for (x = 0; x < LEDCount; x++)
    {
        if (newStatus > x)
        {
            digitalWrite(BasePin + x, HIGH);
        }
        else
        {
            digitalWrite(BasePin + x, LOW);
        }
    }
    oldStatus = newStatus;
}
void LEDStatusStrip::Warning(int WarningLevel)
{
    if (!ledsEnabled) return;
    BlinkEm(WarningLevel, LONG_BLINK);
    delay(1000);
    SetStatus(oldStatus);
}
void LEDStatusStrip::FatalError(int ErrorLevel)
{
    if (!ledsEnabled) return;
    while (1)
    {
        BlinkEm(ErrorLevel, SHORT_BLINK);
        delay(1000);
    }
}
void LEDStatusStrip::BlinkEm(int number, unsigned long duration)
{
    if (!ledsEnabled || LEDCount == 0)
    {
        return;
    }
    int x;
    int y;
    for (y = 0; y < number; y++)
    {
        for (x = 0; x < LEDCount; x++)
        {
            digitalWrite(BasePin + x, HIGH);
        }
        delay(duration/2);
        for (x = 0; x < LEDCount; x++)
        {
            digitalWrite(BasePin + x, LOW);
        }
        delay(duration/2);
    }
}
