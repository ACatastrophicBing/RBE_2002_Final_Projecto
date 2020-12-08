#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"
#include "Median_filter.h"

IRsensor SharpIR;
MedianFilter fillter;

void WallFollowingController::Init(void)
{
    SharpIR.Init();
    fillter.Init();
}

float WallFollowingController::Process(float target_distance)
{
  float time = millis();
  Serial.println(time-prev_time);
  float IR_reading = SharpIR.ReadData();
  float IR_error = IR_reading - target_distance;
  float u = (Kp * IR_error) + (Kd*((IR_error-prev_error)/(time-prev_time)));
  prev_time=time;
  prev_error=IR_error;
  delay(10);
return u;
}