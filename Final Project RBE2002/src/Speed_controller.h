#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

class SpeedController{
    private:
        const float Kp = .3; 
        const float Ki = 0.1; 
        const float Kd = 0.1;
        const float Kpp = .8;
        const float Kip = .2;
        const float Kdp = .1;
        float E_left = 0; 
        float E_right = 0;
        float E_theta = 0; 
        float E_distance = 0;
        float prev_e_theta = 0;
        float prev_e_distance = 0;
        float accel_constraint = 3;//accel_constraint is the maximum oomf that the motor can exert, measured in mm per 50 ms
        float prev_u_left = 0;
        float prev_u_right = 0;
        int counts = 1501.7; //number of counts for a 180 degree turn; you will likely have to change this
        float error_distance = 0;
    public:
        struct constrained_acceleration {
            float constrained_velocity_left;
            float constrained_velocity_right;

        };
        void Init(void);
        void Run(float, float); 
        void ProcessWallFollowSpeed(float, float);
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean TurnNonBlocking(float); //degrees
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean MoveToPosition(float,float); //target_x, target_y
        boolean MoveToPositionBackwards(float,float); //target_x, target_y
        void Stop(void);
        float Constrain(float, float, float);
};

#endif