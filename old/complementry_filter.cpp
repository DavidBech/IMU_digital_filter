#include <math.h>

#include "complementry_filter.hpp"
#include "IMU_CONST.hpp"

complementary_filter::complementary_filter(){
    alpha = 0.05f;
    measure_period_ms = 10;
}

void complementary_filter::set_period_ms(unsigned period_ms){
    measure_period_ms = period_ms;
}

unsigned complementary_filter::get_period_ms(){
    return measure_period_ms;
}

void complementary_filter::set_alpha(float a){
    alpha = a;
}

float complementary_filter::get_alpha(){
    return alpha;
}

float complementary_filter::get_pitch_rad(){
    return pitch_rad;
}

float complementary_filter::get_roll_rad(){
    return roll_rad;
}

float complementary_filter::get_pitch_deg(){
    return get_pitch_rad() * RAD_2_DEG;
}

float complementary_filter::get_roll_deg(){
    return get_roll_rad() * RAD_2_DEG;

}

void complementary_filter::new_measurement(float acceleration[3], float gyro[3]){
    float phiHat_acc_rad = atanf(acceleration[1]/ acceleration[2]);
    float thetaHat_acc_rad = asinf(acceleration[0] / G_MPS2);

    float phiDot_rps = gyro[0] + tanf(pitch_rad) * (sinf(roll_rad) * gyro[1] + cosf(roll_rad) * gyro[2]);
    float thetaDot_rps =                               (cosf(roll_rad) * gyro[1] - sinf(roll_rad) * gyro[2]);

    roll_rad = alpha * phiHat_acc_rad
               + (1.0f - alpha) * (roll_rad + (measure_period_ms / 1000.0f) * phiDot_rps);
  
    pitch_rad = alpha * thetaHat_acc_rad
               + (1.0f - alpha) * (pitch_rad + (measure_period_ms / 1000.0f) * thetaDot_rps);

}

