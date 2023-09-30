#ifndef _motionKepala_H_
#define _motionKepala_H_

#include <Arduino.h>
#include <Soundlibs.h>
#include <main.h>

#define MOTION_KEPALA_IYA       1
#define MOTION_KEPALA_TIDAK     2

class MotionKepala {
    public:

        void Begin();
        uint8_t Motion();
    
    private:

        #define Z_OFFSET        0.08
        #define Y_OFFSET        0.05
        #define SUDUT_GERAKAN   20

        #define DELAY_CALLIBRATION 10

        float Y_tengah = 0.0, Y_atas = 0.0, Y_bawah  = 0.0;
        float Z_tengah, Z_kiri, Z_kanan;
        bool flag_atas_Y = false, flag_bawah_Y = false;
        bool flag_kanan_Z = false, flag_kiri_Z = false;

        float Z_tmp = 0;
        float Y_tmp = 0;

        void reset();

};

#endif