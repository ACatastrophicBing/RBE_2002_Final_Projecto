#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        int threshold = 180;
        int threshold_ramp = 1650;
        long time = 0; //variable to hold the time whenever we want
        int data[3] = {0};//accelerometer
        int data2[3] = {0}; //gyro
        boolean upramp = false; 
        int collisioncounter = 0;
            /*
            Need to run wait 1 second then drive until collision
            Collided -> Wait for button
            Button Pressed -> Back up, turn 90
            90 Turned, Wall Follow
            Vertical Collision going down detected -> Drive Straight 10cm
            10Cm Acheived -> Stop, then kill itself
            */
        enum ROBOT_STATE {IDLE, DRIVE_FOR_COLLISION, DRIVE_FOR_10CM, WALL_FOLLOW, BACK_UP, TURN_90,IDLE_2, DIE_BABY_YODA};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        boolean DetectCollision(void);
        boolean DetectRampEnd(void);
        boolean Drive(float distance, bool dir);//true = straight
};
#endif