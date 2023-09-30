#include <motionKepala.h>

Soundlibs sounds;

void MotionKepala::Begin(){
    // Z_tengah = (qz);
    // Y_tengah = (qy);

    // Z_kanan = (Z_tengah + Z_OFFSET);
    // Z_kiri = (Z_tengah - Z_OFFSET);

    // Y_bawah = Y_tengah - Y_OFFSET;
    // Y_atas = Y_tengah + Y_OFFSET;

}

uint8_t MotionKepala::Motion() {
    uint8_t Jawaban = 0;
    delay(1000);
    reset();
    while(1){
        float Z = yaw;
        float Y = pitch;

        if( (360.0-SUDUT_GERAKAN) < Z_tengah && Z < (0.0+SUDUT_GERAKAN)){
            Z += 360.0;
        } else if( (0.0+SUDUT_GERAKAN) > Z_tengah && Z > (360.0-SUDUT_GERAKAN)){
            Z -= 360.0;
        }

        if( (360.0-SUDUT_GERAKAN) < Y_tengah && Y < (0.0+SUDUT_GERAKAN)){
            Y += 360.0;
        } else if( (0.0+SUDUT_GERAKAN) > Y_tengah && Y > (360.0-SUDUT_GERAKAN)){
            Y -= 360.0;
        }

        if(Z > Z_kanan){
            Z_kanan = Z;
            flag_kanan_Z = true;
        } else if (Z < Z_kiri){
            Z_kiri = Z;
            flag_kiri_Z = true;
        }

        if(Y < Y_bawah){
            Y_bawah = Y;
            flag_bawah_Y = true;
        } else if (Y > Y_atas){
            Y_atas = Y;
            flag_atas_Y = true;
        }

        // Serial.print("| Z|Callibration  ");
        // Serial.print(Z);
        // Serial.print("  |  ");
        // Serial.print(Z_kiri);
        // Serial.print("  +   ");
        // Serial.print(Z_tengah);
        // Serial.print("  +   ");
        // Serial.print(Z_kanan);
        // Serial.println("");
        // Serial.print("| Y|Callibration  ");
        // Serial.print(Y);
        // Serial.print("  |  ");
        // Serial.print(Y_atas);
        // Serial.print("  +   ");
        // Serial.print(Y_tengah);
        // Serial.print("  +   ");
        // Serial.print(Y_bawah);
        // Serial.println("");
        

        if(flag_kanan_Z == true && flag_kiri_Z == true){
            // reset();
            sounds.Play("/sound/command/Jawaban_tidak.mp3");
            Jawaban = MOTION_KEPALA_TIDAK;
            break;
        } else if(flag_atas_Y == true && flag_bawah_Y == true){
            sounds.Play("/sound/command/Jawaban_iya.mp3");
            Jawaban = MOTION_KEPALA_IYA;
            break;
        }
        // delay(1);
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
    reset();
    return Jawaban;
}

void MotionKepala::reset() {
    flag_kanan_Z = false;
    flag_kiri_Z = false;
    flag_atas_Y = false;
    flag_bawah_Y = false;

    // Z_tengah = qz;
    // Y_tengah = qy;

    Z_tengah = yaw;
    Y_tengah = pitch;
    // Z_tmp   = yaw;
    // Y_tmp   = pitch;

    // // Serial.println(Z_tengah);

    Z_kanan = (Z_tengah + (SUDUT_GERAKAN/4));
    Z_kiri = (Z_tengah - (SUDUT_GERAKAN/4));

    Y_bawah = Y_tengah - (SUDUT_GERAKAN/4);
    Y_atas = Y_tengah + (SUDUT_GERAKAN/6);

    // Serial.print("| Z|Callibration  ");
    // // Serial.print(Z);
    // Serial.print(" | ");
    // Serial.print(Z_kiri);
    // Serial.print(" + ");
    // Serial.print(Z_tengah);
    // Serial.print(" + ");
    // Serial.print(Z_kanan);
    // Serial.println("");    

    // Serial.print("| Y|Callibration  ");
    // // Serial.print(Y);
    // Serial.print(" | ");
    // Serial.print(Y_atas);
    // Serial.print(" + ");
    // Serial.print(Y_tengah);
    // Serial.print(" + ");
    // Serial.print(Y_bawah);
    // Serial.println("");  

    // Serial.println("Success");
} 