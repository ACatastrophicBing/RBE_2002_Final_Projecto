#include "IMU.h" 
#include <Romi32U4.h>
#include <LSM6.h>
#include "Median_filter.h"

LSM6 imu;
MedianFilter filter;

void IMU_sensor::Init(void)
{
    Wire.begin();
    if (!imu.init())
    {
        while(1)
        {
            Serial.println("Failed to detect the LSM6.");
            delay(100);
        }
    }
    imu.setFullScaleGyro(imu.GYRO_FS245);
    imu.setFullScaleAcc(imu.ACC_FS2);
    imu.enableDefault();
}

IMU_sensor::acceleration_data IMU_sensor::ReadAcceleration(void)
{
    imu.read();
    return {imu.a.x, imu.a.y, imu.a.z};
}

IMU_sensor::acceleration_data IMU_sensor::ReadGyro(void)
{
    imu.read();
    return {imu.g.x, imu.g.y, imu.g.z};
}

void IMU_sensor::PrintAcceleration(void)
{
    ReadAcceleration();
    snprintf_P(report, sizeof(report),
    PSTR("A: %10d %10d %10d"),
    imu.a.x, imu.a.y, imu.a.z);
    //imu.g.x, imu.g.y, imu.g.z);
    Serial.println(report); 
}

void IMU_sensor::PrintGyro(void)
{
    ReadGyro();
    snprintf_P(report, sizeof(report),
    PSTR("A: %10d %10d %10d"),
    imu.g.x, imu.g.y, imu.g.z);
    //imu.g.x, imu.g.y, imu.g.z);
    Serial.println(report); 
}