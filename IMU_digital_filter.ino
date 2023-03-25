#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SERIAL_PORT Serial

//#define USE_2_SENSORS
#define MATLAB

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
#define AD0_VAL_1 0

double calibration_offsets[2][4];
double calibration_calc[2][4];

ICM_20948_I2C myICM[2]; // Otherwise create an ICM_20948_I2C object

void quat_to_euler(float* angles, double q0, double q1, double q2, double q3);
void process_data(icm_20948_DMP_data_t * data, ICM_20948_I2C* myICM, double* quats);
void print_euler(double* quats, int id);
void calibrate(unsigned sample);
void measure();
void drain();

void setup()
{

  SERIAL_PORT.begin(115200); // Start the serial console

  delay(100);

  while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
  SERIAL_PORT.read();

  SERIAL_PORT.println(F("Press any key to continue..."));

  while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
    ;

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM[0].enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  //myICM[1].enableDebugging();

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.

    myICM[0].begin(WIRE_PORT, AD0_VAL); // WAS AD0_VAL
    #ifdef USE_2_SENSORS
    myICM[1].begin(WIRE_PORT, AD0_VAL_1);
    #endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM[0].statusString());
    SERIAL_PORT.println(myICM[1].statusString());

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
  
  #ifdef USE_2_SENSORS
  success &= (myICM[1].enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
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
  success &= (myICM[0].setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  #ifdef USE_2_SENSORS
  success &= (myICM[1].setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
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
  unsigned drain_length = 1000;
  for(unsigned i=0; i<drain_length; ++i){
    drain();
  }
}

void loop(){
  unsigned calibration_length = 666;
  int read_result;

  calibration_offsets[0][1] = 0;
  calibration_offsets[0][2] = 0;
  calibration_offsets[0][3] = 0;
  calibration_offsets[1][1] = 0;
  calibration_offsets[1][2] = 0;
  calibration_offsets[1][3] = 0;
  calibration_calc[0][1] = 0;
  calibration_calc[0][2] = 0;
  calibration_calc[0][3] = 0;
  calibration_calc[1][1] = 0;
  calibration_calc[1][2] = 0;
  calibration_calc[1][3] = 0;
  while (SERIAL_PORT.available()) SERIAL_PORT.read();
  SERIAL_PORT.println("Wating For Calibration Start");
  do {
    read_result = SERIAL_PORT.read();
  } while (read_result == -1);
  SERIAL_PORT.println(read_result);



  SERIAL_PORT.println("Start Calibration");
  for(unsigned i=1; i<=calibration_length; ++i){
    //calibrate(i);
  }
  
  calibration_offsets[0][1] = calibration_calc[0][1];
  calibration_offsets[0][2] = calibration_calc[0][2];
  calibration_offsets[0][3] = calibration_calc[0][3];
  calibration_offsets[1][1] = calibration_calc[1][1];
  calibration_offsets[1][2] = calibration_calc[1][2];
  calibration_offsets[1][3] = calibration_calc[1][3];

  SERIAL_PORT.println(calibration_offsets[0][1]);
  SERIAL_PORT.println(calibration_offsets[0][2]);
  SERIAL_PORT.println(calibration_offsets[0][3]);
  #ifdef USE_2_SENSORS
  SERIAL_PORT.println(calibration_offsets[1][1]);
  SERIAL_PORT.println(calibration_offsets[1][2]);
  SERIAL_PORT.println(calibration_offsets[1][3]);
  #endif

  SERIAL_PORT.println("Start Exercise");
  while (SERIAL_PORT.available()) SERIAL_PORT.read();
  while(true){
    measure();
    read_result = SERIAL_PORT.read();
    if(read_result != -1){
      break;
    }
  }


  delay(100);

}

void calibrate(unsigned sample){
  double data_double[4];
  icm_20948_DMP_data_t data[2];
  myICM[0].readDMPdataFromFIFO(&data[0]);
  #ifdef USE_2_SENSORS
  myICM[1].readDMPdataFromFIFO(&data[1]);
  #endif

  process_data(&data[0], &myICM[0], 0, data_double);
  calibration_calc[0][1] = (data_double[1] + (calibration_calc[0][1]*sample))/(sample + 1);
  calibration_calc[0][2] = (data_double[2] + (calibration_calc[0][2]*sample))/(sample + 1);
  calibration_calc[0][3] = (data_double[3] + (calibration_calc[0][3]*sample))/(sample + 1);

  #ifdef USE_2_SENSORS
  process_data(&data[1], &myICM[1], 1, data_double);
  calibration_calc[1][1] = (data_double[1] + (calibration_calc[1][1]*sample))/(sample + 1);
  calibration_calc[1][2] = (data_double[2] + (calibration_calc[1][2]*sample))/(sample + 1);
  calibration_calc[1][3] = (data_double[3] + (calibration_calc[1][3]*sample))/(sample + 1);
  #endif
  
  if (myICM[0].status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

void drain(){
  double data_double[4];
  icm_20948_DMP_data_t data[2];
  myICM[0].readDMPdataFromFIFO(&data[0]);
  #ifdef USE_2_SENSORS
  myICM[1].readDMPdataFromFIFO(&data[1]);
  #endif

  if (myICM[0].status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

void measure(){
  double data_double[4];

  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data[2];
  myICM[0].readDMPdataFromFIFO(&data[0]);
  #ifdef USE_2_SENSORS
  myICM[1].readDMPdataFromFIFO(&data[1]);
  #endif

  process_data(&data[0], &myICM[0], 0, data_double);
  print_euler(data_double, 0);
  #ifdef USE_2_SENSORS
  process_data(&data[1], &myICM[1], 1, data_double);
  print_euler(data_double, 1);
  #endif


  if (myICM[0].status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
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

void process_data(icm_20948_DMP_data_t * data, ICM_20948_I2C* myICM, int id, double* quats){
  if ((myICM->status == ICM_20948_Stat_Ok) || (myICM->status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data->header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      quats[1] = ((double)data->Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      quats[2] = ((double)data->Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      quats[3] = ((double)data->Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      quats[1] -= calibration_offsets[id][1];
      quats[2] -= calibration_offsets[id][2];
      quats[3] -= calibration_offsets[id][3];

      quats[0] = sqrt(1.0 - ((quats[1] * quats[1]) + (quats[2] * quats[2]) + (quats[3] * quats[3])));
    }
  }
}

void print_euler(double* quats, int id){
    float angles[3];
    quat_to_euler(angles, quats[0], quats[1], quats[2], quats[3]);

      /*SERIAL_PORT.print(F("Q0:"));
      SERIAL_PORT.print(q0, 3);
      SERIAL_PORT.print(F(" Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.print(q3, 3);*/
      //SERIAL_PORT.print(F(" Accuracy:"));
      //SERIAL_PORT.println(data.Quat9.Data.Accuracy);

    
    /*
      SERIAL_PORT.print(angles[0], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[1], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[2], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[1], 3);

      SERIAL_PORT.write(10);
      SERIAL_PORT.write(13);
      */
    char* angle_names[3];

    #ifdef MATLAB
      if(id == 0){
        angle_names[0] = ("0");
      } else {
        angle_names[0] = ("1");
      }
      /*
      SERIAL_PORT.print(angle_names[0]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[0], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(angles[1], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(angles[2], 3);
      SERIAL_PORT.write(10);
      SERIAL_PORT.write(13);
      */
      SERIAL_PORT.print(angle_names[0]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(quats[0], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(quats[1], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(quats[2], 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(quats[3], 3);
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
