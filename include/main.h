#ifndef _main_h_
#define main_h_


#include <Arduino.h>
#include <bits/stdc++.h> 
#include <Wire.h>
#include <NewPing.h>
#include <motionKepala.h>
#include <cmdTrain.h>
#include <MPU9250lib.h>
#include <algoritma_astar.h>
#include <SD.h>

extern float q0, qx, qy, qz;
extern float pitch, roll, yaw;
extern float heading;
extern float temp;

extern SDFS SD;


#endif