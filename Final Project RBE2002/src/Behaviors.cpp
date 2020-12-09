#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "IMU.h"
#include "Median_filter.h"
#include "Wall_following_controller.h"

//------ Sacred Texts-----------
/* Eat my ass */

/* And titties */
//------------------------

//sensors
Romi32U4ButtonA buttonA;
IMU_sensor LSM6;
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;


//motor-speed controller
SpeedController robot;
WallFollowingController wall_follow;
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
    else{
        robot.Run(50,50);
        return 0;
    }
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    if((abs(data2[0]) < threshold_pick_up) && (abs(data2[1]) < threshold_pick_up) && (abs(data2[2]) < threshold_pick_up)
        && (abs(data2[0]) > 0) && (abs(data2[1]) > 0) && (abs(data2[2]) > 0) ) return 1;
    else return 0;
}

boolean Behaviors::DetectRampEnding(void){
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[2]) < ramp_threshold)){
        Serial.println("Z IMU: ");
        Serial.print(data[2]);
        Serial.println("");
        return 1;
    }
    else{
        return 0;
    }
}

/*boolean Behaviors::Drive(float distance, bool dir){//non blocking version of drive certain distance (mm), make sure to set time to millis() at start of case using this
    float speed = 50;//speed of the robot for driving 10cm in mm/s
    float driveTime = distance/speed;
    if((time + driveTime) <= millis()){//if the time hasn't reached desired time
        if(dir = true){//controls direction so we can do this for both drive 10cm and backup
            robot.Run(speed, speed);
        }
        else{
            robot.Run(-speed, -speed);
        }
        return false;
    }
    else{
        return true;
    }
}*/

/*
boolean straight(float distance){
    return robot.MoveToPosition(distance,0);
}
*/

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
                Serial.println("Drive for Collision");       
            } 
            else { 
                robot_state = IDLE;
                robot.Stop(); 
            }   
            break;
        case IDLE_2://Done, Tested
            if(buttonA.getSingleDebouncedRelease()){ 
                robot_state = BACK_UP; 
                time = millis();//get the current time
                robot.Stop();
                Serial.println("Back up");             
            } 
            break;
        case DRIVE_FOR_COLLISION://Done, untested
            if (DetectCollision()){
                robot_state = IDLE_2;
                robot.Stop();
                robot.Init();
                Serial.println("Collision Detected");
            }else{
                robot.Run(50,50);
            }
            break;
        case DRIVE_FOR_10CM:
            if(buttonA.getSingleDebouncedRelease()){ //If for some reason we have to stop it since its killing itself
                robot_state = IDLE; 
                robot.Stop();             
            } 
            else {
                if(robot.Straight(50,2)){//drive until .1m (10cm) has been reached
                    Serial.println("end");
                    robot_state = IDLE;
                }
            }
            break;
        case TURN_90:
            if(buttonA.getSingleDebouncedRelease()){ //If for some reason we have to stop it since its killing itself
                robot_state = IDLE; 
                robot.Stop();             
            } 
            else {
                if(robot.TurnNonBlocking(-90)){//rotate 90 degrees but using positioning since it turns and gets a move on
                    Serial.println("wall follow");
                    robot.Stop();
                    robot_state = WALL_FOLLOW;
                }
            }

            break;
        case BACK_UP:
            if(buttonA.getSingleDebouncedRelease()){ //If for some reason we have to stop it since its killing itself
                robot_state = IDLE; 
                robot.Stop();             
            } 
            else {
//----------------------- CHANGED TO REVERSE---------------------------
                    robot.Reverse(50,10);
                    Serial.println("Turn 90");
                    robot_state = TURN_90;
                    time = millis();//get the current time
                    robot.Init();
                    break;
                    }
//------------------UPDATED-----------------------
        case WALL_FOLLOW:
            if(DetectRampEnding() == 0){
            wall_follow.Init();
             int speed = wall_follow.Process(30); //distance in [cm]
            robot.ProcessWallFollowSpeed(50-speed,50+speed); //speed in [[mm/s]]
            Serial.println("Following Wall");
            }
            else{
                Serial.println("Ramp End Detected");
                robot.Stop();
                robot_state = DRIVE_FOR_10CM;
                Serial.println("10 cm");
                time = millis();//get the current time
                //no stop since we don't want the object on top to fall off
            }
            break;
            }
}