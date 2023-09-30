#include <cmdTrain.h>

void cmdTrain::begin(String TRAIN_filename_path, String TRAIN_filename_barang){
    _TRAIN_filename_path = TRAIN_filename_path;
    _TRAIN_filename_barang = TRAIN_filename_barang;
    pinMode(pin_led, OUTPUT);
    digitalWrite(pin_led, LOW);
    Serial.println("Train : begin OK");
}

void cmdTrain::getCommand(){
    if(Serial2.available() > 0){
        reading();
        if(tempSerial.toInt() != tujuan){
            tujuan = tempSerial.toInt();
        }
    } else {
        tujuan = 254;
    }
}

void cmdTrain::reading(){
    String tulisan = "";
    if(Serial2.available() > 0){
        while(Serial2.available() > 0){
            char k = (char) Serial2.read();
            if(k=='\n'){
                break;
            }
            tulisan += k;
            vTaskDelay(1/ portTICK_PERIOD_MS);
        }
    }
    tempSerial = tulisan;
    tulisan = "";
}

bool cmdTrain::start_train(){
    getCommand();
    bool back = 0;
    if(tujuan == 99){
        digitalWrite(pin_led, HIGH);
        record_path();
        back = 1;
        akhiran = 1;
    }

    if(tujuan < 5 && tujuan != 99){
        digitalWrite(pin_led, HIGH);
        record_path();
        record_barang();
        back = 1;
        akhiran = 1;
    }

    if(tujuan == 254){
        if(akhiran){
            skat();        
            akhiran = 0;
            ESP.restart();
        }
        back = 0;
        digitalWrite(pin_led, LOW);
    }
    return back;
} 

void cmdTrain::record_path(){
    // Serial.println("Train: record Path");

    dir = SD.open(_TRAIN_filename_path, FILE_APPEND);
    US.update();
    if(dir && US.US_kanan > 1 && US.US_kiri > 1 && US.US_depan > 1 && US.US_belakang > 1){
        if(heading > (258.0-US.US_DERAJAT) && heading < (258.0+US.US_DERAJAT)){ // 270 derajat
            Serial.println("1");
            dir.print(US.US_kanan);
            dir.print(",");
            dir.print(US.US_kiri);
            dir.print(",");
            dir.print(US.US_belakang);
            dir.print(",");
            dir.print(US.US_depan);
            dir.print(",");
            dir.println(heading);
        } else if(heading > (103.0-US.US_DERAJAT) && heading < (103.0+US.US_DERAJAT)){ //90
            Serial.println("2");
            dir.print(US.US_kiri);
            dir.print(",");
            dir.print(US.US_kanan);
            dir.print(",");
            dir.print(US.US_depan);
            dir.print(",");
            dir.print(US.US_belakang);
            dir.print(",");
            dir.println(heading);
        } else if(heading > (188.0-US.US_DERAJAT) && heading < (188.0+US.US_DERAJAT)){
            Serial.println("3");
            dir.print(US.US_belakang);
            dir.print(",");
            dir.print(US.US_depan);
            dir.print(",");
            dir.print(US.US_kiri);
            dir.print(",");
            dir.print(US.US_kanan);
            dir.print(",");
            dir.println(heading);
        } else if((heading > (351.0-US.US_DERAJAT) && heading < 360.0) || (heading > 0.0 && heading < ((351.0+US.US_DERAJAT)-360.0))){
            Serial.println("4");
            dir.print(US.US_depan);
            dir.print(",");
            dir.print(US.US_belakang);
            dir.print(",");
            dir.print(US.US_kanan);
            dir.print(",");
            dir.print(US.US_kiri);
            dir.print(",");
            dir.println(heading);
        }
    }
    dir.close();
    dir = SD.open(_TRAIN_filename_path, FILE_APPEND);
        Serial.print("Path: ");
        Serial.println(dir.size());
    dir.close();
}

void cmdTrain::record_barang(){
    // Serial.println("Train: record barang");

    dir = SD.open(_TRAIN_filename_barang, FILE_APPEND);
    US.update();
    if(dir && US.US_kanan > 1 && US.US_kiri > 1 && US.US_depan > 1 && US.US_belakang > 1){
        if(heading > (258.0-US.US_DERAJAT) && heading < (258.0+US.US_DERAJAT)){ // 270 derajat
            dir.print(US.US_kanan);
            dir.print(",");
            dir.print(US.US_kiri);
            dir.print(",");
            dir.print(US.US_belakang);
            dir.print(",");
            dir.print(US.US_depan);
            dir.print(",");
            dir.print(heading);
            dir.print(",");
            dir.println(tujuan);
        } else if(heading > (103.0-US.US_DERAJAT) && heading < (103.0+US.US_DERAJAT)){ //90
            dir.print(US.US_kiri);
            dir.print(",");
            dir.print(US.US_kanan);
            dir.print(",");
            dir.print(US.US_depan);
            dir.print(",");
            dir.print(US.US_belakang);
            dir.print(",");
            dir.print(heading);
            dir.print(",");
            dir.println(tujuan);
        } else if(heading > (188.0-US.US_DERAJAT) && heading < (188.0+US.US_DERAJAT)){
            dir.print(US.US_belakang);
            dir.print(",");
            dir.print(US.US_depan);
            dir.print(",");
            dir.print(US.US_kiri);
            dir.print(",");
            dir.print(US.US_kanan);
            dir.print(",");
            dir.print(heading);
            dir.print(",");
            dir.println(tujuan);
        } else if((heading > (351.0-US.US_DERAJAT) && heading < 360.0) || (heading > 0.0 && heading < ((351.0+US.US_DERAJAT)-360.0))){
            dir.print(US.US_depan);
            dir.print(",");
            dir.print(US.US_belakang);
            dir.print(",");
            dir.print(US.US_kanan);
            dir.print(",");
            dir.print(US.US_kiri);
            dir.print(",");
            dir.print(heading);
            dir.print(",");
            dir.println(tujuan);
        }
    }
    dir.close();

    dir = SD.open(_TRAIN_filename_barang, FILE_APPEND);
        Serial.println(dir.size());
    dir.close();
}

void cmdTrain::skat(){
    dir = SD.open(_TRAIN_filename_path, FILE_APPEND);
    if(dir){
        dir.print(0);
        dir.print(",");
        dir.print(0);
        dir.print(",");
        dir.print(0);
        dir.print(",");
        dir.print(0);
        dir.print(",");
        dir.println(0);
    }
    dir.close();
}

