#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "IMU.h"
#include "Median_filter.h"

//sensors
Romi32U4ButtonA buttonA;
IMU_sensor LSM6;
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;


//motor-speed controller
SpeedController robot;

void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    robot.Init();
}

boolean Behaviors::DetectCollision(void) //since this goes before detectPickup, this will also act as collection of imu data for both functions
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    
    for(int i = 2; i > 0; i--) data2[i] = data2[i-1];
    data2[0] = data[2];

    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    if((abs(data2[0]) < threshold_pick_up) && (abs(data2[1]) < threshold_pick_up) && (abs(data2[2]) < threshold_pick_up)
        && (abs(data2[0]) > 0) && (abs(data2[1]) > 0) && (abs(data2[2]) > 0) ) return 1;
    else return 0;
}

void Behaviors::Stop(void)
{
    robot.Stop();
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE://Done, Tested
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = DRIVE_FOR_COLLISION;
            delay(1000);//basic little delay here, lets pretend its not blocking ;P
            robot.Stop();             
        } 
        else { 
            robot_state = IDLE;
            robot.Stop(); 
        }   
        break;
    case IDLE_2://Done, Tested
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = TURN_90; 
            robot.Stop();             
        } 
        else { 
            robot_state = IDLE;
            robot.Stop(); 
        }   
        break;
    case DRIVE_FOR_COLLISION://Done, untested
        if (DetectCollision()){
            robot_state = IDLE_2;
            robot.Stop();
            Serial.println("You sunk my battleship!");
        }else{
            robot.Run(50,50);
        }
        break;
    case DRIVE_FOR_10CM://Not finished
        if(buttonA.getSingleDebouncedRelease()){ //If for some reason we have to stop it since its killing itself
            robot_state = IDLE; 
            robot.Stop();             
        } 
        else {
            Serial.println("start");

            Serial.println("end");
            robot_state = IDLE;
        }
        break;
        case TURN_90://Not Finished
            //Back up turn 90

            break;
        case WALL_FOLLOW://Needs to accept the initial bump of going upwards, then go to the next bump and be like AHHH YOU BITCH and run for 10cm
            if(false){

            }else{
                robot_state = DRIVE_FOR_10CM;
                //now just needs to drive straight, easy way is to reset where its positioned, other easy way is to just make a new function to have it drive straight
                //no stop since we don't want the object on top to fall off
            }
            break;
        
    };
}