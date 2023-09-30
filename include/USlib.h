#ifndef _USlib_h_
#define _USlib_h_

#include <Arduino.h>
#include <NewPing.h>
#include <MedianFilterLib.h>
#include <MeanFilterLib.h>
// #include <SingleEMAFilterLib.h>

class USlib{
private:
    /* data */
    #define MEDIAN_NUM      5
    #define MAX_DISTANCE    450     // Maximum distance (in cm) to ping.
    #define SONAR_NUM       4       // Number of sensors.
    NewPing sonar[SONAR_NUM] = {    // Sensor object array.
        NewPing(14, 14, MAX_DISTANCE),  // depan
        NewPing(33, 33, MAX_DISTANCE),  // belakang
        NewPing(32, 32, MAX_DISTANCE),  // kanan
        NewPing(27, 27, MAX_DISTANCE)   // kiri
    };


public:

    float US_DERAJAT = 15.0;
    uint16_t US_depan = 0, US_belakang = 0, US_kanan = 0, US_kiri = 0;
    void update();
};


#endif