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
        //target distance is 2/3 of the distance the IR sensor must sense
        float distance_error_IR = (float)target_distance-fillter.Filter(SharpIR.ReadData());//the filter should filter out all the wack values and make the journey smoother
        float integral = sumE*Ki;
        if(abs(integral)>10);
           integral=integral/abs(integral)*10;
        float distance_speed_IR = distance_error_IR*Kp+(distance_error_IR-prev_e_distance)/20*Kd;//in the IR distance case, I'll only check every 50 ms
        /*Serial.print("Curr: ");
        Serial.println(distance_error_IR);
        Serial.print("Prev: ");
        Serial.println(prev_e_distance);*/
        //Serial.println(distance_speed_IR);
        sumE+=distance_error_IR;
        prev_e_distance = distance_error_IR;//if two sensors are meant to be used at same time, remove this
        Serial.println(distance_speed_IR);
        return distance_speed_IR;
        //return in cm/s
}