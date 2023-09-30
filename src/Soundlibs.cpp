#include "Soundlibs.h"

Soundlibs::Soundlibs(){
    // if (!SD.begin(5, SPI, (F_CPU/2))) {
    //     Serial.println("initialization failed!");
    //     ESP.restart();
    // }

    // if(logger == true){
    //     audioLogger = &Serial;
    // }

    source = new AudioFileSourceSD();
    out = new AudioOutputI2S(0, 1);
    out->SetOutputModeMono(true);
    // out->SetGain(0.1);
    mp3 = new AudioGeneratorMP3();
    
}

void Soundlibs::Play(const char *filename){
    // uint16_t timers = 0;
    while(1){
        if ((mp3) && (mp3->isRunning())) {
            if (!mp3->loop()){
                mp3->stop();
                break;
            }
        } else {
            File file = SD.open(filename);
            if (file) {      
                if (String(file.name()).endsWith(".mp3")) {
                    Serial.println(file.name());
                    source->close();
                    if (source->open(file.name())) { 
                        delay(500);
                        Serial.printf_P(PSTR("Playing '%s' from SD card...\n"), file.name());
                        mp3->begin(source, out);
                    } else {
                        Serial.printf_P(PSTR("Error opening '%s'\n"), file.name());
                    }    
                } 
            } else {
                break;
            }       
        }
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
    return;

}