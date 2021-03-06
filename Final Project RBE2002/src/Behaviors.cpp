#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "IMU.h"
#include "Median_filter.h"
#include "Wall_following_controller.h"

//sensors
Romi32U4ButtonA buttonA;
IMU_sensor LSM6;
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;
WallFollowingController wall_follow;

//motor-speed controller
SpeedController robot;
Romi32U4Motors motors3;

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
    //auto data_gyo = LSM6.ReadGyro();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;

    /*data2[0] = med_x.Filter(data_gyo.X)*0.00875;
    data2[1] = med_y.Filter(data_gyo.Y)*0.00875;
    data2[2] = med_z.Filter(data_gyo.Z)*0.00875;*/

    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else{
        robot.Run(50,50);
        return 0;
    }
}

boolean Behaviors::DetectRampEnd(void){ 
    //auto data_acc = LSM6.ReadAcceleration();
    auto data_gyo = LSM6.ReadGyro();
    //data[2] = med_z.Filter(data_acc.Z)*0.061;
    data2[1] = med_y.Filter(data_gyo.Y);

    if(abs(data2[1]) > threshold_ramp){
    return 1;   }
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
                Serial.println("State 1");       
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
        case DRIVE_FOR_COLLISION://Done, tested
            if (DetectCollision()){
                robot_state = IDLE_2;
                robot.Stop();
                robot.Init();
                Serial.println("Oof Ow Collision Detected");
            }else{
                robot.Run(50,50);
            }
            break;        
        case DRIVE_FOR_10CM://Done, untested
            if(buttonA.getSingleDebouncedRelease()){
                robot_state = IDLE; 
                robot.Stop();             
            } 
            else {
                if(robot.MoveToPosition(.1,0)){//drive until .1m (10cm) has been reached
                    Serial.println("end");
                    robot_state = DIE_BABY_YODA;
                }
            }
            break;
        case TURN_90://Done, untested
            if(buttonA.getSingleDebouncedRelease()){ 
                robot_state = IDLE; 
                robot.Stop();             
            } 
            else {
                if(robot.TurnNonBlocking(-90)){//rotate 90 degrees but using positioning since it turns and gets a move on
                    Serial.println("wall follow");
                    robot.Stop();
                    robot_state = WALL_FOLLOW;
                    time = millis();
                }
            }

            break;
        case BACK_UP://Done, untested
            if(buttonA.getSingleDebouncedRelease()){ 
                robot_state = IDLE; 
                robot.Stop();             
            } 
            else {
                if(robot.MoveToPositionBackwards(-0.1, 0)){//drive backwards
                    Serial.println("Turn 90");
                    robot_state = TURN_90;
                    time = millis();//get the current time
                    robot.Stop();
                }
            }
            break;
        case WALL_FOLLOW://Maybe done, completely untested
            //Needs to accept the initial bump of going upwards, then go to the next bump and be like AHHH YOU BITCH and run for 10cm
            if(DetectRampEnd()&&(time+750)<millis()){//this is the collision detection, for now I'm assuming that it only passes the threshold
            //when it initially is going up the ramp and when its going down the ramp
            //if this isn't the case, all we have to do is use collisioncounter++ and once collisioncounter > 3, we end it
                collisioncounter++;
                upramp = true;
                time = millis();
                Serial.println("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
                Serial.println(collisioncounter);
            }
            if(!upramp){//This boolean has to check if its gone up the ramp, topped at the ramp, then gone down the ramp, and then turns true once
            //the robot hits the floor one last time, we have to figure out if the threshold is reached when the robot goes up the ramp or tops out
            //If upramp is false, it will continue wall following, if upramp is true, it will end
            //Also, the code to change it to function when all 4 instances of going up or down are collisions is just if(collisioncounte<4)
           
                int speed = wall_follow.Process(40); //distance in [cm]
                robot.Run(50-speed,50+speed); //speed in [[mm/s]]
                LSM6.PrintGyro();

            }
            else{
                robot.Stop();
                robot_state = DRIVE_FOR_10CM;
                Serial.println("10 cm");
                time = millis();//get the current time
                //no stop since we don't want the object on top to fall off
            }
            break;
        
        case DIE_BABY_YODA:
            //Throws baby yoda off the robot to prove he is not glued :(
            long end_time = millis()+20000;
            while(millis()<end_time){
                motors3.setEfforts(0,0);
            }

            robot.Turn(90, 0);
            
            long end_time2 = millis() + 1000;
            while(millis()<end_time2){
                motors3.setEfforts(300,300);
            }
            motors3.setEfforts(0,0);
            robot_state = IDLE;
            break;
    };
}