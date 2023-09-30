#ifndef _Soundlibs_H_
#define _Soundlibs_H_

// #ifndef _Arduino_h_
#include <Arduino.h>

#include "AudioFileSourceSD.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include "AudioFileSourceID3.h"
// #include "SD.h"
#include <main.h>

class Soundlibs {
private:

    #define logger false

    AudioGeneratorMP3 *mp3      = NULL;
    AudioFileSourceSD *source   = NULL;
    AudioOutputI2S *out         = NULL;
    AudioFileSourceID3 *id3;

    File dir;

    void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string);

public:
    Soundlibs();
    void Play(const char *filename);
    
};

#endif