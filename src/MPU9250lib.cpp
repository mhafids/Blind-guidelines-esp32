#include "MPU9250lib.h"

MeanFilter<float> MnF_Heading(Median_Mean_Size+10);
MeanFilter<float> MnF_qz(Median_Mean_Size);
MeanFilter<float> MnF_qx(Median_Mean_Size);
MeanFilter<float> MnF_qy(Median_Mean_Size);
MeanFilter<float> MnF_q0(Median_Mean_Size);

SingleEMAFilter<float> EMA_Heading(EMA_size);
SingleEMAFilter<float> EMA_q0(EMA_size);
SingleEMAFilter<float> EMA_qx(EMA_size);
SingleEMAFilter<float> EMA_qy(EMA_size);
SingleEMAFilter<float> EMA_qz(EMA_size);

void MPU9250lib::Begin(){

  if (imu.begin() != INV_SUCCESS){
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
      ESP.restart();
    }
  }
  
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(16); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(98); // Set LPF corner frequency to 5Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(100); // Set mag rate to 10Hz

  imu.dmpBegin( DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              100
              ); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive


  uint32_t startsmillis = millis();
  Serial.println("callibrate");
  while(1){
    // Check for new data in the FIFO
    if ( imu.fifoAvailable() ){
      // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
      if ( imu.dmpUpdateFifo() == INV_SUCCESS)
      {
        // computeEulerAngles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        imu.computeEulerAngles(false);
        if((WAKTU_KALIBRASI * 1000) < (millis() - startsmillis)){
          break;
        }
      }
    }

    // dataReady() checks to see if new accel/gyro data
    // is available. It will return a boolean true or false
    // (New magnetometer data cannot be checked, as the library
    //  runs that sensor in single-conversion mode.)
    if ( imu.dataReady() )
    {
      // Call update() to update the imu objects sensor data.
      // You can specify which sensors to update by combining
      // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
      // UPDATE_TEMPERATURE.
      // (The update function defaults to accel, gyro, compass,
      //  so you don't have to specify these values.)
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      // printIMUData();
    }
  }
  Serial.println("callibrate selesai");
}

void MPU9250lib::Update(){
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() ){
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      printIMUData();
    }
  }

  // dataReady() checks to see if new accel/gyro data
  // is available. It will return a boolean true or false
  // (New magnetometer data cannot be checked, as the library
  //  runs that sensor in single-conversion mode.)
  if ( imu.dataReady() ){
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS | UPDATE_TEMP);
    printIMUData();
  }

}

void MPU9250lib::printIMUData(void){  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  q0      = imu.calcQuat(imu.qw);
  qx      = imu.calcQuat(imu.qx);
  qy      = imu.calcQuat(imu.qy);
  qz      = imu.calcQuat(imu.qz);
  mx      = imu.calcMag(imu.mx);
  my      = imu.calcMag(imu.my);
  mz      = imu.calcMag(imu.mz);
  pitch   = imu.pitch;
  roll    = imu.roll;
  yaw     = imu.yaw;
  heading = imu.computeCompassHeading();

  

  // Serial.println(temperatureRead());

  // Serial.println("Mag: " + String(magX) + ", " +
  //             String(magY) + ", " + String(magZ) + " uT");
  // Serial.println("Q: " + String(q0, 4) + ", " +
  //                   String(q1, 4) + ", " + String(q2, 4) + 
  //                   ", " + String(q3, 4));
  // Serial.println("R/P/Y: " + String(imu.roll) + ", "
  //           + String(imu.pitch) + ", " + String(imu.yaw));
  // Serial.println("Time: " + String(imu.time) + " ms");
  // Serial.println();


  MnF_Heading.AddValue(heading);
  heading = MnF_Heading.GetFiltered();
  EMA_Heading.AddValue(heading);
  heading = EMA_Heading.GetLowPass();

  MnF_q0.AddValue(q0);
  q0 = MnF_q0.GetFiltered();
  EMA_q0.AddValue(q0);
  q0 = EMA_q0.GetLowPass();

  MnF_qx.AddValue(qx);
  qx = MnF_qx.GetFiltered();
  EMA_qx.AddValue(qx);
  qx = EMA_qx.GetLowPass();

  MnF_qy.AddValue(qy);
  qy = MnF_qy.GetFiltered();
  EMA_qy.AddValue(qy);
  qy = EMA_qy.GetLowPass();

  MnF_qz.AddValue(qz);
  qz = MnF_qz.GetFiltered();
  EMA_qz.AddValue(qz);
  qz = EMA_qz.GetLowPass();

}

