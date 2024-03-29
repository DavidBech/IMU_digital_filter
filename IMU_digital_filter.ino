#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SERIAL_PORT Serial

#define USE_2_SENSORS

#define MATLAB

#define WIRE_PORT Wire // Your desired Wire port.

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
#define AD0_VAL_1 0

int64_t calibration_calc[2][4];
int32_t calibration_calc_32[2][4];
double calibration_quat[2][4];

ICM_20948_I2C myICM[2];

void quat_to_euler(float* angles, double q0, double q1, double q2, double q3);
bool process_data(icm_20948_DMP_data_t * data, ICM_20948_I2C* myICM, double* quats, double* other_data);
void print_euler(double* quats, double* other_data, int id);
bool calibrate(int id, unsigned sample);
void measure();
bool drain();
void multiply_quats(double* result, double*q0, double* q1);
void quats_fixed_to_double(double* result, uint32_t* fixed);

void setup(){
    SERIAL_PORT.begin(115200); // Start the serial console

    delay(100);

    // Make sure the serial RX buffer is empty
    while (SERIAL_PORT.available()) SERIAL_PORT.read();

    SERIAL_PORT.println(F("Press any key to continue..."));
    
    while (!SERIAL_PORT.available());
    //TODO: should we consume these characters

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    //myICM[0].enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
    //myICM[1].enableDebugging();

    bool initialized = false;
    while (!initialized){
      // Initialize the ICM-20948
      // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
      myICM[0].begin(WIRE_PORT, AD0_VAL);
      #ifdef USE_2_SENSORS
      myICM[1].begin(WIRE_PORT, AD0_VAL_1);
      #endif

      SERIAL_PORT.print(F("Initialization of the sensor returned: "));
      SERIAL_PORT.println(myICM[0].statusString());
      #ifdef USE_2_SENSORS
      SERIAL_PORT.println(myICM[1].statusString());
      #endif

      #ifdef USE_2_SENSORS
      if (myICM[0].status != ICM_20948_Stat_Ok || myICM[1].status != ICM_20948_Stat_Ok)
      #else 
      if (myICM[0].status != ICM_20948_Stat_Ok)
      #endif
      {

        SERIAL_PORT.println(F("Trying again..."));
        delay(500);
      } else {
        initialized = true;
      }
    }

    SERIAL_PORT.println(F("Device connected!"));

    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM[0].initializeDMP() == ICM_20948_Stat_Ok);

    #ifdef USE_2_SENSORS
    success &= (myICM[1].initializeDMP() == ICM_20948_Stat_Ok);
    #endif
    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    success &= (myICM[0].enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (myICM[0].enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (myICM[0].enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
    //success &= (myICM[0].enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (myICM[0].enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);

    #ifdef USE_2_SENSORS
    success &= (myICM[1].enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (myICM[1].enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (myICM[1].enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
    #endif

    // Enable any additional sensors / features
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (myICM[0].setDMPODRrate(DMP_ODR_Reg_Quat9, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[0].setDMPODRrate(DMP_ODR_Reg_Accel, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[0].setDMPODRrate(DMP_ODR_Reg_Gyro, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[0].setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[0].setDMPODRrate(DMP_ODR_Reg_Cpass, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[0].setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 5) == ICM_20948_Stat_Ok); // Set to the maximum

    #ifdef USE_2_SENSORS
    success &= (myICM[1].setDMPODRrate(DMP_ODR_Reg_Quat9, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[1].setDMPODRrate(DMP_ODR_Reg_Accel, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[1].setDMPODRrate(DMP_ODR_Reg_Gyro, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[1].setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[1].setDMPODRrate(DMP_ODR_Reg_Cpass, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM[1].setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 5) == ICM_20948_Stat_Ok); // Set to the maximum
    #endif

    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (myICM[0].enableFIFO() == ICM_20948_Stat_Ok);
    #ifdef USE_2_SENSORS
    success &= (myICM[1].enableFIFO() == ICM_20948_Stat_Ok);
    #endif

    // Enable the DMP
    success &= (myICM[0].enableDMP() == ICM_20948_Stat_Ok);
      #ifdef USE_2_SENSORS
    success &= (myICM[1].enableDMP() == ICM_20948_Stat_Ok);
    #endif

    // Reset DMP
    success &= (myICM[0].resetDMP() == ICM_20948_Stat_Ok);
    #ifdef USE_2_SENSORS
    success &= (myICM[1].resetDMP() == ICM_20948_Stat_Ok);
    #endif 

    // Reset FIFO
    success &= (myICM[0].resetFIFO() == ICM_20948_Stat_Ok);
    #ifdef USE_2_SENSORS
    success &= (myICM[1].resetFIFO() == ICM_20948_Stat_Ok);
    #endif

    // Check success
    if (success){
      SERIAL_PORT.println(F("DMP enabled!"));
    } else {
      SERIAL_PORT.println(F("Enable DMP failed!"));
      SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
      while (1); // Do nothing more
    }

    SERIAL_PORT.println("Start Drain");
    unsigned drain_length =500;
    //unsigned drain_length = 0;
    unsigned i = 0;
    while(i < drain_length){
        if(!drain()){
          continue;
        }

        ++i;
    }
}

void loop(){
    //unsigned calibration_length = 2900;
    unsigned calibration_length = 50;

    int read_result = -1;
    bool success = true; 

    calibration_calc[0][1] = 0;
    calibration_calc[0][2] = 0;
    calibration_calc[0][3] = 0;
    calibration_calc[1][1] = 0;
    calibration_calc[1][2] = 0;
    calibration_calc[1][3] = 0;

    while (SERIAL_PORT.available()) SERIAL_PORT.read();
    SERIAL_PORT.println("Waiting For Calibration Start");
    do {
      read_result = SERIAL_PORT.read();
    } while (read_result == -1);
    SERIAL_PORT.println(read_result);

    // Reset FIFO
    success &= (myICM[0].resetFIFO() == ICM_20948_Stat_Ok);
    #ifdef USE_2_SENSORS
    success &= (myICM[1].resetFIFO() == ICM_20948_Stat_Ok);
    #endif

    //if(!success){
    //    SERIAL_PORT.println("Reset FIFO Failed");
    //}

    SERIAL_PORT.println("Start Calibration");
    unsigned sample_0, sample_1;
    sample_0 = 0;
    sample_1 = 0;
    #ifdef USE_2_SENSORS
    while(sample_0 < calibration_length || sample_1 < calibration_length){
    #else
    while(sample_0 < calibration_length){
    #endif

        bool sample_0_pass = 0;
        bool sample_1_pass = 0;
        sample_0_pass = calibrate(0, sample_0);
        #ifdef USE_2_SENSORS
        sample_1_pass = calibrate(1, sample_1);
        #endif

        if(sample_0_pass){
          ++sample_0;
        }
        if(sample_1_pass){
          ++sample_1;
        }

        if(!(sample_0_pass || sample_1_pass)){
          delay(10);
        }
    }
  
    calibration_calc_32[0][1] = (int32_t)calibration_calc[0][1];
    calibration_calc_32[0][2] = (int32_t)calibration_calc[0][2];
    calibration_calc_32[0][3] = (int32_t)calibration_calc[0][3];
    
    
    SERIAL_PORT.print(calibration_calc_32[0][1]);
    SERIAL_PORT.print(" ");
    SERIAL_PORT.print(calibration_calc_32[0][2]);
    SERIAL_PORT.print(" ");
    SERIAL_PORT.print(calibration_calc_32[0][3]);
    SERIAL_PORT.print(" ");
    SERIAL_PORT.println();
    

    calibration_quat[0][1] = (double)(calibration_calc_32[0][1]) / 1073741824.0;
    calibration_quat[0][2] = (double)(calibration_calc_32[0][2]) / 1073741824.0;
    calibration_quat[0][3] = (double)(calibration_calc_32[0][3]) / 1073741824.0;

    // calibration_quat[0][1] = (double)(calibration_calc[0][1]) / 1073741824.0;
    // calibration_quat[0][2] = (double)(calibration_calc[0][2]) / 1073741824.0;
    // calibration_quat[0][3] = (double)(calibration_calc[0][3]) / 1073741824.0;

    calibration_quat[0][0] = sqrt(1.0 - ((calibration_quat[0][1] * calibration_quat[0][1]) + (calibration_quat[0][2] * calibration_quat[0][2]) + (calibration_quat[0][3] * calibration_quat[0][3])));

    // SERIAL_PORT.println("Calibration Value1");
    // SERIAL_PORT.println(calibration_quat[0][1]);
    // SERIAL_PORT.println(calibration_quat[0][2]);
    // SERIAL_PORT.println(calibration_quat[0][3]);
    // SERIAL_PORT.println(calibration_quat[0][0]);

    
    calibration_calc_32[1][1] = (int32_t)calibration_calc[1][1];
    calibration_calc_32[1][2] = (int32_t)calibration_calc[1][2];
    calibration_calc_32[1][3] = (int32_t)calibration_calc[1][3];

    
    SERIAL_PORT.print(calibration_calc_32[1][1]);
    SERIAL_PORT.print(" ");
    SERIAL_PORT.print(calibration_calc_32[1][2]);
    SERIAL_PORT.print(" ");
    SERIAL_PORT.print(calibration_calc_32[1][3]);
    SERIAL_PORT.println();
    

    calibration_quat[1][1] = (double)(calibration_calc_32[1][1]) / 1073741824.0;
    calibration_quat[1][2] = (double)(calibration_calc_32[1][2]) / 1073741824.0;
    calibration_quat[1][3] = (double)(calibration_calc_32[1][3]) / 1073741824.0;
    calibration_quat[1][0] = sqrt(1.0 - ((calibration_quat[1][1] * calibration_quat[1][1]) + (calibration_quat[1][2] * calibration_quat[1][2]) + (calibration_quat[1][3] * calibration_quat[1][3])));

    // SERIAL_PORT.println("Calibration Value2");
    // SERIAL_PORT.println(calibration_quat[1][1]);
    // SERIAL_PORT.println(calibration_quat[1][2]);
    // SERIAL_PORT.println(calibration_quat[1][3]);
    // SERIAL_PORT.println(calibration_quat[1][0]);

    // quats_fixed_to_double(&calibration_quat[0][0], &calibration_calc_32[0][0]);
    // quats_fixed_to_double(&calibration_quat[1][0], &calibration_calc_32[1][0]);

    SERIAL_PORT.println("Start Exercise");
    while (SERIAL_PORT.available()) SERIAL_PORT.read();
  
    while(true){
        measure();
        read_result = SERIAL_PORT.read();
        if(read_result != -1){
          break;
        }
    }


    //delay(100);
}

bool calibrate(int i, unsigned sample){
    icm_20948_DMP_data_t sensor_data;
    ICM_20948_Status_e status;

    status = myICM[i].readDMPdataFromFIFO(&sensor_data);
    if(status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail){
          if ((sensor_data.header & DMP_header_bitmap_Quat9) > 0) {

            calibration_calc[i][1] = (sensor_data.Quat9.Data.Q1 + (calibration_calc[i][1]*sample))/(sample + 1);
            calibration_calc[i][2] = (sensor_data.Quat9.Data.Q2 + (calibration_calc[i][2]*sample))/(sample + 1);
            calibration_calc[i][3] = (sensor_data.Quat9.Data.Q3 + (calibration_calc[i][3]*sample))/(sample + 1);
            
            SERIAL_PORT.print(i);
            SERIAL_PORT.print(" ");
            SERIAL_PORT.print(sensor_data.Quat9.Data.Q1);
            SERIAL_PORT.print(" ");
            SERIAL_PORT.print(sensor_data.Quat9.Data.Q2);
            SERIAL_PORT.print(" ");
            SERIAL_PORT.print(sensor_data.Quat9.Data.Q3);
            SERIAL_PORT.println();
            return true;
          }
    }
    return false;
}

bool drain(){
    icm_20948_DMP_data_t data;
    ICM_20948_Status_e status;
    bool wait = 1;

    #ifdef USE_2_SENSORS
    for(int i = 0; i < 2; ++i){
    #else
    int i = 0;
    #endif

        status = myICM[i].readDMPdataFromFIFO(&data);
        if(status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail){
            wait = 0;
        } else {
            wait &= 1; 
        }


    #ifdef USE_2_SENSORS
    }
    #endif

    if(wait) {
      delay(10);
      return false;
    }
    return true;
}

void measure(){
    double quats[4];
    double other_data[10];
    bool wait = 1;

    memset(&other_data[0], 0, sizeof(other_data));

    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    icm_20948_DMP_data_t sensor_data[2];
    ICM_20948_Status_e status;

    #ifdef USE_2_SENSORS
    for(int i = 0; i < 2; ++i){
    #else
    int i = 0;
    #endif

        status =  myICM[i].readDMPdataFromFIFO(&sensor_data[i]);
        if(status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail){
          bool got_quats = process_data(&sensor_data[i], &myICM[i], i, quats, other_data);
          if(got_quats){
            print_euler(quats, other_data, i);
            wait = 0;
          } else {
            wait &= 1;
          }
        } else {
          wait &= 1;
        }

    #ifdef USE_2_SENSORS
    }
    #endif

    if (wait){
      delay(10);
    }
}

void quat_to_euler(float* angles, double q0, double q1, double q2, double q3){
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    angles[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q0 * q2 - q1 * q3));
    double cosp = sqrt(1 - 2 * (q0 * q2 - q1 * q3));
    angles[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    angles[2] = atan2(siny_cosp, cosy_cosp);

    angles[0] *= 180.0f/3.14159265358979323846f;
    angles[1] *= 180.0f/3.14159265358979323846f;
    angles[2] *= 180.0f/3.14159265358979323846f;

}

// void quats_fixed_to_double(double* result, uint32_t* fixed){
//     result[1] = (double)fixed[1] / 1073741824.0;
//     result[2] = (double)fixed[2] / 1073741824.0;
//     result[3] = (double)fixed[3] / 1073741824.0;
//     result[0] = sqrt(1.0 - ((result[1] * result[1]) + (result[2] * result[2]) + (result[3] * result[3])));
// }

void multiply_quats(double* result, double*q0, double* q1){
    // result[0] = q0[0]*q1[0] - q0[1]*q1[1] - q0[2]*q1[2] - q0[3]*q1[3];
    // result[1] = q0[0]*q1[1] + q0[1]*q1[0] + q0[2]*q1[3] - q0[3]*q1[2];
    // result[2] = q0[0]*q1[2] - q0[1]*q1[3] + q0[2]*q1[0] + q0[3]*q1[1];
    // result[3] = q0[0]*q1[3] + q0[1]*q1[2] - q0[2]*q1[1] + q0[3]*q1[0];

    // z1 = a+bi+cj+dk
    // z2 = e+fi+gj+hk
    // z1 * z2= (a*e - b*f - c*g- d*h) + i (b*e + a*f + c*h - d*g) + j (a*g - b*h + c*e + d*f) + k (a*h + b*g - c*f + d*e)
    
    //Correctly multiplying by the conjugate
    result[0] = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];
    result[1] = -q0[0]*q1[1] + q0[1]*q1[0] - q0[2]*q1[3] + q0[3]*q1[2];
    result[2] = -q0[0]*q1[2] + q0[1]*q1[3] + q0[2]*q1[0] - q0[3]*q1[1];
    result[3] = -q0[0]*q1[3] - q0[1]*q1[2] + q0[2]*q1[1] + q0[3]*q1[0];
    
}

bool process_data(icm_20948_DMP_data_t * data, ICM_20948_I2C* myICM, int id, double* quats, double* other_data){
  bool quat_data = 0;
  if ((myICM->status == ICM_20948_Stat_Ok) || (myICM->status == ICM_20948_Stat_FIFOMoreDataAvail))  {
    // We have asked for orientation data so we should receive Quat9
    if ((data->header & DMP_header_bitmap_Quat9) > 0) {
        double temp[4];
        uint32_t quats_fixed[4];

        //quats[1] = (double)(data->Quat9.Data.Q1 - calibration_offsets[id][1]) / 1073741824.0;
        //quats[2] = (double)(data->Quat9.Data.Q2 - calibration_offsets[id][2]) / 1073741824.0;
        //quats[3] = (double)(data->Quat9.Data.Q3 - calibration_offsets[id][3]) / 1073741824.0;
        temp[1] = (double)(data->Quat9.Data.Q1) / 1073741824.0;
        temp[2] = (double)(data->Quat9.Data.Q2) / 1073741824.0;
        temp[3] = (double)(data->Quat9.Data.Q3) / 1073741824.0;
        temp[0] = sqrt(1.0 - ((temp[1] * temp[1]) + (temp[2] * temp[2]) + (temp[3] * temp[3])));


        // quats[0] = temp[0];
        // quats[1] = temp[1];
        // quats[2] = temp[2];
        // quats[3] = temp[3];
        // SERIAL_PORT.println("QuatData");
        // SERIAL_PORT.println(temp[1]);
        // SERIAL_PORT.println(temp[2]);
        // SERIAL_PORT.println(temp[3]);
        // SERIAL_PORT.println(temp[0]);

        // quats_fixed[1] = data->Quat9.Data.Q1;
        // quats_fixed[2] = data->Quat9.Data.Q2;
        // quats_fixed[3] = data->Quat9.Data.Q3;
        // quats_fixed_to_double(&temp[0], &quats_fixed[0]);

        // todo: Multiplication is NON commutative we need to check if this order is correct
        //multiply_quats(quats, calibration_quat[id], temp); //&temp[0]);
        multiply_quats(quats, temp, calibration_quat[id]); //&temp[0]);

        quat_data = 1;
    } 
    if((data->header & DMP_header_bitmap_Compass_Calibr) > 0){
        other_data[3] = ((double)data->Compass_Calibr.Data.X)/(65536.0);
        other_data[4] = ((double)data->Compass_Calibr.Data.Y)/(65536.0);
        other_data[5] = ((double)data->Compass_Calibr.Data.Z)/(65536.0);
    } 
    if((data->header & DMP_header_bitmap_Gyro_Calibr) > 0){

    } 
    if((data->header & DMP_header_bitmap_Geomag) > 0){

    } 
    if((data->header & DMP_header_bitmap_Gyro) > 0){

    } 
    if((data->header & DMP_header_bitmap_Accel) > 0){
        other_data[0] = (double)data->Raw_Accel.Data.X;
        other_data[1] = (double)data->Raw_Accel.Data.Y;
        other_data[2] = (double)data->Raw_Accel.Data.Z;
    }


  } 
  return quat_data;  
}

void print_euler(double* quats, double* other_data, int id){
    char angle_names[3][10];
    float angles[3];

    // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
    // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
    // The quaternion data is scaled by 2^30.

    quat_to_euler(angles, quats[0], quats[1], quats[2], quats[3]);

    #ifdef MATLAB
      if(id == 0){
        angle_names[0][0] = '0';
        angle_names[0][1] = '\0';
      } else {
        angle_names[0][0] = '1';
        angle_names[0][1] = '\0';
      }
      
      SERIAL_PORT.print(angle_names[0]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[0], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[1], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[2], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(other_data[0], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(other_data[1], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(other_data[2], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(other_data[3], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(other_data[4], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(other_data[5], 3);
      SERIAL_PORT.println();
      //SERIAL_PORT.write(10);
      //SERIAL_PORT.write(13);
      /*
      SERIAL_PORT.print(angle_names[0]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(quats[0], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(quats[1], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(quats[2], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(quats[3], 3);
      */
    #else 
    if(id == 0){
      angle_names[0] = (" roll0:");
      angle_names[1] = (" pitch0:");
      angle_names[2] = (" yaw0:");
    } else {
      angle_names[0] = (" roll1:");
      angle_names[1] = (" pitch1:");
      angle_names[2] = (" yaw1:");
    }

    SERIAL_PORT.print(angle_names[0]);
    SERIAL_PORT.print(angles[0], 3);
    SERIAL_PORT.print(angle_names[1]);
    SERIAL_PORT.print(angles[1], 3);
    SERIAL_PORT.print(angle_names[2]);
    SERIAL_PORT.println(angles[2], 3);
    #endif
}
