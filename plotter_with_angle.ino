#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "complementry_filter.hpp"

Adafruit_MPU6050 mpu;

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

float phiHat_rad = 0.0f;
float thetaHat_rad = 0.0f;

void loop() {
  double G_MPS2 = 9.81;
  double alpha = 0.05f;
  double rad_2_deg = 57.2957795131f; 
  int delta_t_ms = 10;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float phiHat_acc_rad = atanf(a.acceleration.y/ a.acceleration.z);
  float thetaHat_acc_rad = asinf(a.acceleration.x / G_MPS2);

  float phiDot_rps = g.gyro.x + tanf(thetaHat_rad) * (sinf(phiHat_rad) * g.gyro.y + cosf(phiHat_rad) * g.gyro.z);
  float thetaDot_rps =                               (cosf(phiHat_rad) * g.gyro.y - sinf(phiHat_rad) * g.gyro.z);

  phiHat_rad = alpha * phiHat_acc_rad
               + (1.0f - alpha) * (phiHat_rad + (delta_t_ms / 1000.0f) * phiDot_rps);
  
  thetaHat_rad = alpha * thetaHat_acc_rad
               + (1.0f - alpha) * (thetaHat_rad + (delta_t_ms / 1000.0f) * thetaDot_rps);
  
  float phi_deg = phiHat_rad * rad_2_deg;
  float theta_deg = thetaHat_rad * rad_2_deg;


  /* Print out the values */
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
  */
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);
  Serial.print(",");
  Serial.print("AnglePhi:");
  Serial.print(phi_deg);
  Serial.print(",");
  Serial.print("AngleTheta:");
  Serial.print(theta_deg);
  Serial.print(",");
  Serial.print("Zero:");
  Serial.println(0);
  delay(delta_t_ms);

  
}