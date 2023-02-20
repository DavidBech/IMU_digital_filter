#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "ICM_20948.h" 

#include "complementry_filter.hpp"
#include "kalman_filter.hpp"

complementary_filter comp_filter;
kalman_filter kal_filter;

#define NINEAXIS

#ifdef NINEAXIS
ICM_20948_I2C myICM;
#define SERIAL_PORT Serial

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#endif

#ifdef SIXAIXS
Adafruit_MPU6050 mpu;
#endif


void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  #ifdef SIXAXIS
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  #endif

  #ifdef NINEAXIS
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  bool initialized = false;
  while (!initialized){
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)  {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else{
      initialized = true;
    }
  }

  #endif

  Serial.println("");
  delay(100);
}


void loop() {
  unsigned delay_ms = 300;
  float comp_filter_alpha = 0.05f;
  float accel[3];
  float gyro[3];

  comp_filter.set_period_ms(delay_ms);
  comp_filter.set_alpha(comp_filter_alpha);
  /* Get new sensor events with the readings */

  #ifdef NINEAXIS
  if (myICM.dataReady()) {
    myICM.getAGMT(); 
    //printScaledAGMT(&myICM);
    accel[0] = myICM.accX() *9.81f/1000.0f;
    accel[1] = myICM.accY() *9.81f/1000.0f;
    accel[2] = myICM.accZ() *9.81f/1000.0f;
    gyro[0] = myICM.gyrX() *3.14f/180.0f;
    gyro[1] = myICM.gyrY() *3.14f/180.0f;
    gyro[2] = myICM.gyrZ() *3.14f/180.0f;

    delay(delay_ms);
  } else {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }

  #endif

  #ifdef SIXAXIS
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel[0] = a.acceleration.x;
  accel[1] = a.acceleration.y;
  accel[2] = a.acceleration.z;
  gyro[0] = g.gyro.x;
  gyro[1] = g.gyro.y;
  gyro[2] = g.gyro.z;
  #endif

  comp_filter.new_measurement(accel, gyro);
  
  kal_filter.new_measure(accel, gyro);


  /*
  Serial.print("AccelX:");
  Serial.print(accel[0]);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accel[1]);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accel[2]);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(gyro[0]);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(gyro[1]);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(gyro[2]);
  Serial.print(",");
  */
  Serial.print("CompPitchAngle:");
  Serial.print(comp_filter.get_pitch_deg());
  Serial.print(",");
  Serial.print("CompRollAngle:");
  Serial.print(comp_filter.get_roll_deg());
  Serial.print(",");
  Serial.print("KalPitchAngle:");
  Serial.print(kal_filter.get_pitch_deg());
  Serial.print(",");
  Serial.print("KalRollAngle:");
  Serial.print(kal_filter.get_roll_deg());
  Serial.print(",");
  
  Serial.print("Zero:");
  Serial.print(0);
  Serial.print(",");
  Serial.print("Ninety:");
  Serial.print(90);
  Serial.print(",");
  Serial.print("NegNinety:");
  Serial.println(-90);
  delay(delay_ms);
  
}