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
  float ir_val = analogRead(pin_IR);
  float x = (1/ir_val);
 // float x = (ir_val);
  //Serial.println(x);
  delay(50);
//y=5798*x+-1.73
  float ir_cm_y = (((5798*x)-1.73));
  return ir_cm_y;
}