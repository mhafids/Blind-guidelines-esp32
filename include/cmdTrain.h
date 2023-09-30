#ifndef _cmdTrain_h_
#define _cmdTrain_h_

#include <Arduino.h>
#include <main.h>
#include <USlib.h>
// #include <config_global.h>

class cmdTrain
{
private:
    /* data */
    #define pin_led 2

    USlib US;

    String tempSerial;
    uint8_t tujuan = 254;    
    File dir;

    String _TRAIN_filename_path;
    String _TRAIN_filename_barang;

    bool akhiran = 0;

    void reading();
    void record_path();
    void record_barang();
    void getCommand();
    void skat();

public:
    void begin(String TRAIN_filename_path, String TRAIN_filename_barang);
    bool start_train();
};


#endif 