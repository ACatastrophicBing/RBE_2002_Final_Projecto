#ifndef WALL_FOLLOWING_CONTROLLER
#define WALL_FOLLOWING_CONTROLLER

#include <Romi32U4.h>

class WallFollowingController{
    private:
        const float Kp = 3; //Adapt parameters Kp and Kd until your robot consistently drives along a wall
        const float Kd = 0.6;
        float prev_time = 0;
        float prev_error = 0;

    public:
        void Init(void);
        float Process(float);
};

#endif