#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  float reading = (float)1/analogRead(pin_IR);
  
  float speed = (float)5469*reading + 0.589;//slope of the line I calculate from calibration
  return speed;
}