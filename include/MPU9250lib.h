#ifndef _MPU9250lib_H_
#define _MPU9250lib_H_

#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>
#include "MedianFilterLib.h"
#include <MeanFilterLib.h>
#include <SingleEMAFilterLib.h>

class MPU9250lib {
private:
    #define WAKTU_KALIBRASI 15 // 15 detik kalibrasi
    #define Median_Mean_Size 10
    #define EMA_size 0.6

    void printIMUData(void);
    MPU9250_DMP imu;

public:
    // #define North 351.0
    // #define South 188.0
    // #define West 258.0
    // #define East 103.0
    const float North = 345.0;
    const float South = 184.0;
    const float West = 262.0;
    const float East = 95.0;

    float q0, qx, qy, qz;
    float rate, heading;
    float temp;
    float mx, my, mz;
    float pitch, roll, yaw;
    
    void Begin();
    void Update();


};

#endif