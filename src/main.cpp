#include <main.h>
#include <config.h>
#include <MedianFilterLib.h>

using namespace std;

TaskHandle_t Task1;
TaskHandle_t Task2;
MotionKepala motion_kepala;
Soundlibs sound;
cmdTrain cmdtrain;
MPU9250lib mpu;
USlib _US_;
algoritma_astar Astar;
MedianFilter<int16_t> X_med(10);
MedianFilter<int16_t> Y_med(10);

uint8_t * tujuan = new uint8_t[3];
float q0, qx, qy, qz;
float pitch, roll, yaw;
float heading;
float temp;
bool kalibrasi_semua = 0;

vector<vector<int16_t>> path;
vector<int16_t> USmod;

void Petunjuk_penggunaan();
vector<uint32_t> Pertanyaannya(vector<vector<int32_t>> file_tujuan);
void Task1code( void * pvParameters );
void Task2code( void * pvParameters );
void US_TASK( void * pvParameters );
void US_test( void * pvParameters );
void Heading_test( void * pvParameters );
void Jalan_test( void * pvParameters );
vector<vector<int16_t>> arahgerak(vector<vector<int16_t>> Paths);
vector<int16_t> US_rotate();
int jalanjalan(vector<int16_t> US_mod, int i);

String kamar[4] = {
    "Meja_dan_Kursi",
    "Kasur",
    "Lemari",
    "Pintu"
};
bool trainmode = 0;


String TRAIN_filename_path = "/cache/.map_kamar_rmh";
String TRAIN_filename_barang = "/cache/.map_barang_rmh";
// int16_t Panjang_Kamar = 500, Lebar_Kamar = 240;
int16_t Panjang_Kamar = 293, Lebar_Kamar = 242;

// Mode 0 : Normal
// Mode 1 : US test
// Mode 2 : Heading test
// Mode 3 : Jalan test
#define Mode 0

void setup() {
  Serial.begin(115200);
  Serial2.begin(2000000);

  // Selamat Datang 
  if(Mode == 0){
    if (!SD.begin(5, SPI, (F_CPU/2))) {
        Serial.println("initialization failed!");
        ESP.restart();
    }
  }

  if(Mode == 0){
    xTaskCreatePinnedToCore(
      US_TASK, /* Task function. */
      "US task",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      100,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */

    xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      100000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      100,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */
    
    xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      100,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 0 */

  } else if(Mode == 1){

    xTaskCreatePinnedToCore(
      US_test, /* Task function. */
      "US test",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */

    xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      100,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 0 */

  } else if(Mode == 2){

    xTaskCreatePinnedToCore(
      Heading_test, /* Task function. */
      "Heading Test",   /* name of task. */
      1000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */
    
    xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      100,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 0 */

  } else if(Mode == 3){

    xTaskCreatePinnedToCore(
      Jalan_test, /* Task function. */
      "Jalan Test",   /* name of task. */
      20000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */
    
    xTaskCreatePinnedToCore(
      US_TASK, /* Task function. */
      "US task",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */

    xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      100,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 0 */

  }
}

void loop() {
}

void Task1code( void * pvParameters ){
  // sound.Play("/sound/command/selamat-datang.mp3");
  sound.Play("/sound/command/kalibrasi.mp3");

  motion_kepala.Begin();
  cmdtrain.begin(TRAIN_filename_path, TRAIN_filename_barang);
  Astar.init(TRAIN_filename_path, Panjang_Kamar, Lebar_Kamar);

  while(1){
    // cmdtrain.start_train(); 

    if(kalibrasi_semua == 1){
      break;
    }
    vTaskDelay(1/ portTICK_PERIOD_MS);
    delay(1);
  }

  sound.Play("/sound/command/kalibrasi_selesai.mp3");

  // Petunjuk_penggunaan();
  // bool tanya = 0;
  vector<uint32_t> tujuan;
  for(;;){
    // bool training = cmdtrain.start_train(); 

    // Serial.print(xPortGetCoreID());
    // Serial.print('\t');
    // Serial.println(heading);
    // delay(1000);

    // Petunjuk_penggunaan();
    // motion_kepala.Motion();

    if(tujuan.size() == 0){
      vector<vector<int32_t>> file_tujuan = Astar.Tujuan_koordinat(TRAIN_filename_barang, TRAIN_filename_path);
      tujuan = Pertanyaannya(file_tujuan);
    }

    int tempPemanggil = 99;
    if(tujuan.size() > 0){
      int utama_berhenti = 0;
      int8_t a = Astar.Search(TRAIN_filename_path, USmod[0], USmod[1], tujuan[0], tujuan[1]);
      if(a > 0){
        switch (a){
          case 1:
            sound.Play("/sound/command/astar_posisi_pengguna_tidak_ketemu.mp3");
            break;

          case 2:
            sound.Play("/sound/command/astar_posisi_tujuan_tidak_ketemu.mp3");
            break;

          case 3:
            sound.Play("/sound/command/astar_posisi_tujuan_tidak_ketemu.mp3");
            break;

          case 4:
            sound.Play("/sound/command/astar_posisi_pengguna_tidak_ketemu.mp3");
            break;

          case 5:
            sound.Play("/sound/command/Sampai_tujuan.mp3");
            break;

        }

        if(a == 5){
          tujuan.clear();
        }
        // Astar.Search(US_mod[0], US_mod[1], tujuan[0], tujuan[1]);
        continue;
      }

      path = arahgerak(Astar.Paths);    
      for (auto i : path){
        for (auto j : i){
          Serial.print(j);          
          Serial.print(" ");          
        }
        Serial.println();
      }

      for (int i = 0; i < path.size(); i++){
        if(path[i][2] == 0){
          utama_berhenti = 1;
          sound.Play("/sound/command/Sampai_tujuan.mp3");
          tujuan.clear();
          break;
        }

        // Serial.print("US X: ");
        // Serial.print(USmod[0]);
        // Serial.print("+");
        // Serial.print(path[i+1][0]);
        // Serial.print("\t");
        // Serial.print("US Y: ");
        // Serial.print(USmod[1]);
        // Serial.print("+");
        // Serial.print(path[i+1][1]);
        // Serial.print("\t");
        // Serial.print("Heading: ");
        // Serial.print(heading);
        // Serial.println();

        if(path[i+1][0] != USmod[0] || path[i+1][1] != USmod[1]){  
          bool semua_berhenti = 0;

          for (int m = 0; m < path.size(); m++){
            if(path[m][0] == USmod[0] && path[m][1] == USmod[1]){
              i = m; 
              break;
            }

            if(m == path.size()-1){
              semua_berhenti = 1;
              break;
            }
          }

          if(semua_berhenti){
            break;
          }
        }

        int aa = jalanjalan(USmod, i);
        if(aa != tempPemanggil){
          switch (aa){
            case -1:
              /* code */
              sound.Play("/sound/arah/Berputar ke kiri.mp3");
              tempPemanggil = aa;
              break;

            case 1:
              /* code */
              sound.Play("/sound/arah/Berputar ke kanan.mp3");
              tempPemanggil = aa;
              break;

            case 0:
              /* code */
              sound.Play("/sound/arah/maju.mp3");
              tempPemanggil = aa;
              break;

          }
        }

        vTaskDelay(1/ portTICK_PERIOD_MS);
      }                        

    }
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }
}

void Task2code( void * pvParameters ){
  // // sensor callibrate
  mpu.Begin();
  kalibrasi_semua = 1;
  for(;;){
    mpu.Update(); 
    heading = mpu.heading;
    temp = mpu.temp;
    qz = mpu.qz;
    qy = mpu.qy;
    qx = mpu.qx;
    q0 = mpu.q0;
    pitch = mpu.pitch;
    roll = mpu.roll;
    yaw = mpu.yaw;
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }
}

void Petunjuk_penggunaan(){
  sound.Play("/sound/command/petunjuk penggunaan 1.mp3");
  sound.Play("/sound/command/petunjuk penggunaan 2.mp3");
  sound.Play("/sound/command/petunjuk penggunaan 3.mp3");
  uint8_t y = motion_kepala.Motion();
  if(y == MOTION_KEPALA_TIDAK){
    Petunjuk_penggunaan();
  }
}

vector<uint32_t> Pertanyaannya(vector<vector<int32_t>> file_tujuan){
  sound.Play("/sound/command/Tujuan.mp3");
  uint8_t kamar_jum = (sizeof(kamar)/sizeof(kamar[0]));
  vector<uint32_t> jum_barang;
  vector<vector<uint32_t>> barang_XY;

  for (auto i : file_tujuan){
    for (auto k : i){
      Serial.print(k);
      Serial.print(" ");
    }    
    Serial.println();
  }  

  for (int i = 0; i < kamar_jum; i++){

    uint8_t how = 0;
    vector<uint32_t> X;
    vector<uint32_t> Y;
    for (auto j : file_tujuan){
      if(j[2] == i){
        X.push_back(j[0]);
        Y.push_back(j[1]);
        how++;
      }
    }

    if(how == 0){
      continue;
    }

    if(how != 0){
      vector<uint32_t> cx;
      sort(X.begin(), X.end());
      sort(Y.begin(), Y.end());
      uint16_t X_n = floor(X.size()/2);
      uint16_t Y_n = floor(Y.size()/2);
      Serial.println(X_n);
      uint32_t X_f = 0;
      uint32_t Y_f = 0;

      if(X_n == 0){
        X_f = X[X_n];        
      } else if(X_n%2 == 0){
        X_f = (X[X_n] + X[X_n+1])/2;
      } else if(X_n%2 == 1){
        X_f = X[X_n];
      }

      if(Y_n == 0){
        Y_f = Y[Y_n];        
      } else if(Y_n%2 == 0){
        Y_f = (Y[Y_n] + Y[Y_n+1])/2;
      } else if(Y_n%2 == 1){
        Y_f = Y[Y_n];
      }


      bool cek_X = 0;
      for (size_t k = 0; k < file_tujuan.size(); k++){
        /* code */
        vector<int32_t> j = file_tujuan[k];
        if(j[0] == X_f && j[2] == i){
          X_f = j[0];
          Y_f = j[1];
          break;
        }
        if( (file_tujuan.size()-1) == k){
          cek_X = 1;
        }
      }
      
      if(cek_X){
        for (size_t k = 0; k < file_tujuan.size(); k++){
          /* code */
          vector<int32_t> j = file_tujuan[k];
          if(j[1] == Y_f && j[2] == i){
            X_f = j[0];
            Y_f = j[1];
            break;
          }
        }
      }


      cx.push_back(X_f);
      cx.push_back(Y_f);
      jum_barang.push_back(i);
      barang_XY.push_back(cx);
    }
  }
  

  Serial.println("Jum barang");
  for (auto i : jum_barang){
    Serial.println(i);
  }  

  Serial.println("barang XY");
  for (auto i : barang_XY){
    for (auto k : i){
      Serial.print(k);
      Serial.print(" ");
    }    
    Serial.println();
  }  

  // for (auto k : barang_XY){
  //   Serial.print(k);
  //   Serial.print(" ");
  // }    
  // Serial.println();

  int8_t numTJ = 0;
  for (int i = 0; i < jum_barang.size(); i++){
    String ruangan = "/sound/kamar/";
    int8_t kalimat = jum_barang[i];
    ruangan.concat(kamar[kalimat]);
    ruangan.concat(".mp3");
    char suara[(ruangan.length()+1)];
    ruangan.toCharArray(suara, (ruangan.length()+1));
    sound.Play(suara);
    uint8_t y = motion_kepala.Motion();
    if(y == MOTION_KEPALA_IYA){
      numTJ = i;
      break;
    }

    if(jum_barang.size()-1 == i){
      i = -1;
    }
  }

  return barang_XY[numTJ];

}

vector<int16_t> US_rotate(){
  vector<int16_t> m;
  _US_.update();

  // Serial.print("1. kanan\t");
  // Serial.print("1. kiri\t");
  // Serial.print("1. belakang\t");
  // Serial.println("1. depan\t");
  // Serial.print(_US_.US_kanan);
  // Serial.print("\t");
  // Serial.print(_US_.US_kiri);
  // Serial.print("\t");
  // Serial.print(_US_.US_belakang);
  // Serial.print("\t");
  // Serial.println(_US_.US_depan); 

  
    // Serial.print(heading);
    // Serial.print("\t");
    // Serial.print(_US_.US_kanan);
    // Serial.print("\t");
    // Serial.print(_US_.US_kiri);
    // Serial.print("\t");
    // Serial.print(_US_.US_belakang);
    // Serial.print("\t");
    // Serial.println(_US_.US_depan); 
    // Serial.println("---------------------------------"); 
    
  vector<uint16_t> ultrasonic;
  if(heading > (mpu.West-_US_.US_DERAJAT) && heading < (mpu.West+_US_.US_DERAJAT)){ // west 
      ultrasonic.push_back(_US_.US_kanan); // atas
      ultrasonic.push_back(_US_.US_kiri); // bawah
      ultrasonic.push_back(_US_.US_belakang); // kanan
      ultrasonic.push_back(_US_.US_depan); // kiri
  } else if(heading > (mpu.East-_US_.US_DERAJAT) && heading < (mpu.East+_US_.US_DERAJAT)){ //90
      ultrasonic.push_back(_US_.US_kiri);
      ultrasonic.push_back(_US_.US_kanan);
      ultrasonic.push_back(_US_.US_depan);
      ultrasonic.push_back(_US_.US_belakang);
  } else if(heading > (mpu.South-_US_.US_DERAJAT) && heading < (mpu.South+_US_.US_DERAJAT)){
      ultrasonic.push_back(_US_.US_belakang);
      ultrasonic.push_back(_US_.US_depan);
      ultrasonic.push_back(_US_.US_kiri);
      ultrasonic.push_back(_US_.US_kanan);
  } else if((heading > (mpu.North-_US_.US_DERAJAT) && heading < 360.0) || (heading > 0.0 && heading < ((mpu.North+_US_.US_DERAJAT)-360.0))){
      ultrasonic.push_back(_US_.US_depan);
      ultrasonic.push_back(_US_.US_belakang);
      ultrasonic.push_back(_US_.US_kanan);
      ultrasonic.push_back(_US_.US_kiri);
  }    

  if(ultrasonic.size() == 0){
    return m;
  }

  // Serial.print(heading);
  // Serial.print("\t");
  // Serial.print(ultrasonic[0]);
  // Serial.print("\t");
  // Serial.print(ultrasonic[1]);
  // Serial.print("\t");
  // Serial.print(ultrasonic[2]);
  // Serial.print("\t");
  // Serial.println(ultrasonic[3]); 

  // Serial.print(Astar.X_max);
  // Serial.print("\t");
  // Serial.print(Astar.Y_max);
  // Serial.print("\t");
  // Serial.println(Astar.Tinggi_Muka);

  if(ultrasonic[2] == 1 && ultrasonic[3] == 1){
    return m;
  } else if(ultrasonic[2] <= 1){
    m.push_back((int) (ultrasonic[3]  / Astar.Tinggi_Muka) );
  } else if(ultrasonic[3] <= 1){
    m.push_back( (int) ((Astar.X_max - ultrasonic[2]) / Astar.Tinggi_Muka) );
  } else if(ultrasonic[2] < ultrasonic[3]){ // X
    m.push_back( (int) ((Astar.X_max - ultrasonic[2]) / Astar.Tinggi_Muka) );
  } else if(ultrasonic[2] >= ultrasonic[3]) {
    m.push_back((int) (ultrasonic[3]  / Astar.Tinggi_Muka) );
  }   
    
  if(ultrasonic[0] == 1 && ultrasonic[1] == 1){
    return m;
  } else if(ultrasonic[0] <= 1){
    m.push_back( (int) (ultrasonic[1] / Astar.Tinggi_Muka) );
  } else if(ultrasonic[1] <= 1){
    m.push_back( (int) ((Astar.Y_max - ultrasonic[0]) / Astar.Tinggi_Muka) );
  } else if(ultrasonic[0] > ultrasonic[1]){ // Y
    m.push_back( (int) (ultrasonic[1] / Astar.Tinggi_Muka) );
  } else if(ultrasonic[0] < ultrasonic[1]) {
    m.push_back( (int) ((Astar.Y_max - ultrasonic[0]) / Astar.Tinggi_Muka) );
  }

  // Serial.print(m[0]);
  // Serial.print("\t");
  // Serial.print(m[1]);
  // Serial.print("\t");
  // Serial.println(m.size());


  return m;
}

vector<vector<int16_t>> arahgerak(vector<vector<int16_t>> Paths){

	vector<vector<int16_t>> mmy;
	for (size_t i = 0; i < Paths.size(); i++){
		/* code */
		vector<int16_t> mk;
		if(Paths.size()-1 == i){
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(0);
		} else if(Paths[i][0] > Paths[i+1][0] && Paths[i][1] == Paths[i+1][1]){ // kiri / west / barat 
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(4);
		} else if(Paths[i][0] < Paths[i+1][0] && Paths[i][1] == Paths[i+1][1]){  // kanan /east / timur 
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(3);		
		} else if(Paths[i][0] == Paths[i+1][0] && Paths[i][1] > Paths[i+1][1]){ // bawah / south / selatan
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(2);		
		} else if(Paths[i][0] == Paths[i+1][0] && Paths[i][1] < Paths[i+1][1]){ // atas / north / utara
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(1);		
		}
		mmy.push_back(mk);
	}

  return mmy;
}

int jalanjalan(vector<int16_t> US_mod, int i){
	float AngleJalan = 25.0;
	int exec = 99;
	if(path[i][2] == 1){ //cek arah north 
    if((heading > (mpu.North-AngleJalan) && heading < 360.0) || (heading > 0.0 && heading < ((mpu.North+AngleJalan)-360.0))){
			//play music  
			exec = 0;
		} else {
			//play music
			if( ((mpu.North-AngleJalan) > heading || (360.0-(mpu.North+AngleJalan)) > heading) && heading > mpu.South){ // 325 > H > 188.0 = kiri
				exec = 1;
			} else if( (((mpu.North+AngleJalan)-360.0) < heading || (mpu.North+AngleJalan) < heading) < heading && heading < mpu.South){ // 0 < H < 188.0 = kanan
				exec = -1;
			}
		}

	} else if(path[i][2] == 2){//cek arah south
    if((mpu.South-AngleJalan) < heading && heading < (mpu.South+AngleJalan)){ // 184 - 204
			//play music 
			exec = 0;
		} else {
			//play music
			if((mpu.South-AngleJalan) > heading && (heading > mpu.North || heading > 0)){ // 178.0 > H > -15 = kanan
				exec = 1;
			} else if((mpu.South+AngleJalan) < heading && (heading < mpu.North || heading < 360.0)){ // 198 < H < 360 = kiri
				exec = -1;
			}
		}

	} else if(path[i][2] == 3){//cek arah east
    if(heading > (mpu.East-AngleJalan) && heading < (mpu.East+AngleJalan)){
			//play music
			exec = 0;
		} else {
			//play music
			if( (mpu.East+AngleJalan) < heading && heading < mpu.West ){ // 113 < 258.0 = kiri
				exec = -1;
			} else if( ((mpu.South-AngleJalan) > heading && heading > 0.0) || (360.0 > heading && heading > mpu.West) ){ // 93 > 0/360 > 258.0 = kanan
				exec = 1;
			}
		} 

	} else if(path[i][2] == 4){//cek arah west
    if((mpu.West-AngleJalan) < heading && heading < (mpu.West+AngleJalan)){ // 242 - 262 - 282
			//play music
			exec = 0;
		} else {
      //play music
			if( (mpu.West-AngleJalan) > heading && heading > mpu.East){ // 248 > 103 = kanan
				exec = 1;
			} else if( ((mpu.West+AngleJalan) < heading && heading < 360.0) || (0.0 < heading && heading < mpu.East) ){ // 268 < 0/360 < 103 = kiri
				exec = -1;
			}
    }
	}

  return exec;
}

void US_test( void * pvParameters ){
  // Astar.init(TRAIN_filename_path, 279, 242);
  Astar.init(TRAIN_filename_path, Panjang_Kamar, Lebar_Kamar);

  while(1){
    // cmdtrain.start_train(); 

    if(kalibrasi_semua == 1){
      break;
    }
    vTaskDelay(1/ portTICK_PERIOD_MS);
    delay(1);
  }

  for(;;){
    // _US_.update();
    // Serial.print(_US_.US_belakang);
    // Serial.print("-");
    // Serial.print(_US_.US_depan);
    // Serial.print("-");
    // Serial.print(_US_.US_kanan);
    // Serial.print("-");
    // Serial.println(_US_.US_kiri);

    vector<int16_t> US_mod = US_rotate();  
    if(US_mod.size() == 0){
      vTaskDelay(1/ portTICK_PERIOD_MS);
      continue;
    } 
    // for (auto k : US_mod){
    //   Serial.print(k);
    //   Serial.print(" ");
    // }
    // Serial.println("=================");    
    US_mod[0] = X_med.AddValue(US_mod[0]);
    US_mod[1] = Y_med.AddValue(US_mod[1]);

    for (auto k : US_mod){
      Serial.print(k);
      Serial.print("\t");
    }
    Serial.print(Astar.Tinggi_Muka);
    Serial.println("");
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }

}

void US_TASK( void * pvParameters ){

  while(1){
    // cmdtrain.start_train(); 

    if(kalibrasi_semua == 1){
      break;
    }
    vTaskDelay(1/ portTICK_PERIOD_MS);
    delay(1);
  }

  Serial.println("US TASK: done");
  USmod.push_back(0);
  USmod.push_back(0);

  for(;;){
    vector<int16_t> USmods = US_rotate();  
    if(USmods.size() == 0){
      vTaskDelay(1/ portTICK_PERIOD_MS);
      continue;
    } 

    USmod[0] = X_med.AddValue(USmods[0]);
    USmod[1] = Y_med.AddValue(USmods[1]);

    vTaskDelay(1/ portTICK_PERIOD_MS);
  }

}

void Heading_test( void * pvParameters ){

  while(1){
    // cmdtrain.start_train(); 

    if(kalibrasi_semua == 1){
      break;
    }
    vTaskDelay(1/ portTICK_PERIOD_MS);
    delay(1);
  }


  for(;;){
    Serial.println(heading);
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }

}

void Jalan_test( void * pvParameters ){
  Astar.init(TRAIN_filename_path, Panjang_Kamar, Lebar_Kamar);

  for (size_t i = 0; i < 4; i++){
    /* code */
    vector<int16_t>km; 
    km.push_back(1);
    km.push_back(1);
    km.push_back(i+1);
    path.push_back(km);
    
  }
  
  while(1){
    // cmdtrain.start_train(); 

    if(kalibrasi_semua == 1){
      break;
    }
    vTaskDelay(1/ portTICK_PERIOD_MS);
    delay(1);
  }

  int i = 0;
  for(;;){
      /* code */
    int aa = jalanjalan(USmod, i);
    Serial.print("US X: ");
    Serial.print(USmod[0]);
    Serial.print("+");
    Serial.print(path[i][0]);
    Serial.print("\t");
    Serial.print("US Y: ");
    Serial.print(USmod[1]);
    Serial.print("+");
    Serial.print(path[i][1]);
    Serial.print("\t");
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.print("+");
    Serial.print(path[i][2]);
    Serial.print("\t");
    Serial.print("Exec: ");
    Serial.println(aa);


  // i++;
  // if(i == path.size()-1){
  //   i = 0;
  // }

  delay(300);
  vTaskDelay(1/portTICK_RATE_MS);
  }
}
