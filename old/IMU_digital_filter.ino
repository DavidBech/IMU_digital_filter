#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "complementry_filter.hpp"
#include "kalman_filter.hpp"

Adafruit_MPU6050 mpu;
complementary_filter comp_filter;
kalman_filter kal_filter;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

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
  Serial.println("");
  delay(100);
}


void loop() {
  unsigned delay_ms = 10;
  float comp_filter_alpha = 0.05f;
  float accel[3];
  float gyro[3];

  comp_filter.set_period_ms(delay_ms);
  comp_filter.set_alpha(comp_filter_alpha);
  /* Get new sensor events with the readings */

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel[0] = a.acceleration.x;
  accel[1] = a.acceleration.y;
  accel[2] = a.acceleration.z;
  gyro[0] = g.gyro.x;
  gyro[1] = g.gyro.y;
  gyro[2] = g.gyro.z;

  comp_filter.new_measurement(accel, gyro);
  
  kal_filter.new_measure(accel, gyro);


  /*
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);
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