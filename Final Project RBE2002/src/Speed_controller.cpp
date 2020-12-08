#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Position_estimation.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;

float time_track = 0;

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    odometry.Init();
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        time_track = time_track + 50/1000.0;
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        u_left = prev_u_left + Constrain(u_left-prev_u_left,-accel_constraint,accel_constraint);
        u_right = prev_u_right + Constrain(u_right-prev_u_right,-accel_constraint,accel_constraint);

        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(target_velocity_left,target_velocity_right);

        prev_u_left = u_left;
        prev_u_right = u_right;

    }
}

boolean SpeedController::MoveToPosition(float target_x, float target_y)
{
    if(error_distance>= 0.01)
    {    
        float e_x = odometry.ReadPose().X - target_x;
        float e_y = odometry.ReadPose().Y - target_y;
        float e_theta;

        if(e_y == 0 && e_x == 0){
            e_theta = 0;
        }
        else{
            e_theta = odometry.ReadPose().THETA - atan2(-e_y, -e_x);
        }
        
        error_distance = sqrt(pow(e_x, 2) + pow(e_y,2));

        E_theta += e_theta;
        E_distance += error_distance;

        float ui_theta = Constrain(Kip*E_theta,-20,20);
        float ui_distance = Constrain(Kip*E_distance,-15,15);

        float ud_theta = Constrain(10*Kdp*(e_theta-prev_e_theta),-10,10);
        float ud_distance = Constrain(10*Kdp*(error_distance-prev_e_distance),-10,10);

        float u_theta = Kpp*180*e_theta+ui_theta+ud_theta;
        float u_distance = Kpp*100*error_distance+ui_distance+ud_distance;

        float u_left = Constrain(u_distance + u_theta, -50.0, 50.0);
        float u_right = Constrain(u_distance - u_theta, -50.0, 50.0);

        prev_e_distance = error_distance;
        prev_e_theta = e_theta;

        Run(u_left, u_right);//do thing at speed zoom
        
    } else{
    E_theta = 0; //reset error info for next positioning
    E_distance = 0;
    return 1;
    }
    return 0;//now can go to next state pretty much
}

float SpeedController::Constrain(float value, float min, float max)
{
    if(value < min){
        return min;
    }
    else if(value > max){
        return max;
    }
    return value;
}

boolean SpeedController::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);
    int turns = counts*(degree/180.0);
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        if(!direction) Run(50,-50);
        else Run(-50,50);
    }
    
    return 1;
}

boolean SpeedController::TurnNonBlocking(float degree)
{
    if(error_distance>= 0.01)
    {    
        float e_theta = odometry.ReadPose().THETA - degree;

        E_theta += e_theta;

        float ui_theta = Constrain(Kip*E_theta,-20,20);

        float ud_theta = Constrain(10*Kdp*(e_theta-prev_e_theta),-10,10);

        float u_theta = Kpp*180*e_theta+ui_theta+ud_theta;

        float u_left = Constrain(u_theta, -50.0, 50.0);
        float u_right = Constrain(-u_theta, -50.0, 50.0);

        prev_e_theta = e_theta;

        Run(u_left, u_right);//do thing at speed zoom
        
    } else{
    E_theta = 0; //reset error info for next positioning
    return 1;
    }
    return 0;//now can go to next state pretty much
}

boolean SpeedController::Straight(int target_velocity, int time)
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();
    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
    }
    while(MagneticEncoder.ReadVelocityLeft()!=0 || MagneticEncoder.ReadVelocityRight()!=0){
        Run(0,0);
    }
    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
    odometry.Stop();
    time_track = 0;
    E_left = 0;
    E_right = 0;
    E_theta = 0;
    E_distance = 0;
}
