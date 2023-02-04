#include "complementry_filter.hpp"

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

float complementary_filter::get_yaw_rad(){
    return yaw_rad;
}

float complementary_filter::get_roll_rad(){
    return roll_rad;
}

float complementary_filter::get_yaw_deg(){
    return get_yaw_rad() * RAD_2_DEG;
}

float complementary_filter::get_roll_deg(){
    return get_roll_rad() * RAD_2_DEG;

}

void complementary_filter::new_measurement(float acceleration[3], float gyro[3]){
    
}

