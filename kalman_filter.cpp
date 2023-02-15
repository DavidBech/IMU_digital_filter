#include <BasicLinearAlgebra.h>
#include <math.h>

#include "IMU_CONST.hpp"
#include "kalman_filter.hpp"

kalman_filter::kalman_filter(){
	X_state_estimate(0)= 0;
	X_state_estimate(1)= 0;

	F_state_trans(0, 0) = 1;
	F_state_trans(1, 0) = 0;

	P_Error_Cov(0,0) = 0.0f;
	P_Error_Cov(0,1) = 0.0f;
	P_Error_Cov(1,0) = 0.0f;
	P_Error_Cov(1,1) = 0.0f;

	Q_Cov_Noise(0,0) = 0.01f;
	Q_Cov_Noise(0,1) = 0.00f;
	Q_Cov_Noise(1,0) = 0.01f;
	Q_Cov_Noise(1,1) = 0.00f;

	R_Cov_Noise(0,0) = 0.01f;
	R_Cov_Noise(0,1) = 0.00f;
	R_Cov_Noise(0,2) = 0.00f;
	R_Cov_Noise(1,0) = 0.00f;
	R_Cov_Noise(1,1) = 0.01f;
	R_Cov_Noise(1,2) = 0.00f;
	R_Cov_Noise(2,0) = 0.00f;
	R_Cov_Noise(2,1) = 0.00f;
	R_Cov_Noise(2,2) = 0.01f;

    sampleTime_s = (10.0f) / 1000.0f; // 10ms
}

void kalman_filter::new_measure(float accel[3], float gyro[3]){
	BLA::Matrix<3> Gyro_Data;
	Gyro_Data(0) = gyro[0];
	Gyro_Data(1) = gyro[1];
	Gyro_Data(2) = gyro[2];

	predict(&Gyro_Data);

    ++measure_count;
    if(measure_count == predicts_per_update){
		BLA::Matrix<3> Accel_Data;
		Accel_Data(0) = accel[0];
		Accel_Data(1) = accel[1];
		Accel_Data(2) = accel[2];
        update(&Accel_Data);
        measure_count = 0;
    }
}

void kalman_filter::predict(BLA::Matrix<3> *Gyro_Data){
	predict_update_state(Gyro_Data);

	predict_update_error_covariance(Gyro_Data);
}

void kalman_filter::predict_update_state(BLA::Matrix<3> *Gyro_Data){
	// x(n+1) = x(n) + t * f(x(n), u)
	calc_F_state_trans();
	X_state_estimate = X_state_estimate + (F_state_trans * *Gyro_Data) * sampleTime_s;
}

void kalman_filter::calc_F_state_trans(){
	F_state_trans(0, 1) = sin(X_state_estimate(0)) * tan(X_state_estimate(1));
	F_state_trans(0, 2) = cos(X_state_estimate(0)) * tan(X_state_estimate(1));
	F_state_trans(1, 1) = cos(X_state_estimate(0));
	F_state_trans(1, 2) = sin(X_state_estimate(0));
}

void kalman_filter::calc_Jacobian_of_F(BLA::Matrix<3> *Gyro_Data, BLA::Matrix<2,2>*A_in){
	BLA::Matrix<3> &GD = *Gyro_Data;
	BLA::Matrix<2,2> &A = *A_in;
	float sin_phi = sinf(X_state_estimate(0));
	float cos_phi = cosf(X_state_estimate(0));
	float sec_the = 1.0f / cosf(X_state_estimate(1));
	float tan_the = tanf(X_state_estimate(1));
	A(0,0) = (GD(0)*cos_phi - GD(2)*sin_phi) * tan_the; // d phi / d phi
	A(0,1) = (GD(0)*sin_phi	+ GD(2)*cos_phi) * sec_the * sec_the; // d phi / d theta
	A(1,0) = -GD(0)*sin_phi - GD(1)*cos_phi; // d theta / d phi
	A(1,1) = 0.0f; // d theta / d theta
}

void kalman_filter::predict_update_error_covariance(BLA::Matrix<3> *Gyro_Data){
	// P(n+1) = P(n) + t(A P(n) + P(n) ~A + Q)
	// A is Jacobian of F, ~A is transposition of A
	BLA::Matrix<2,2> A_Jacob_F;
	calc_Jacobian_of_F(Gyro_Data, &A_Jacob_F);

	P_Error_Cov = P_Error_Cov + (A_Jacob_F * P_Error_Cov * ~A_Jacob_F + Q_Cov_Noise) * sampleTime_s; //ansley
}

void kalman_filter::update(BLA::Matrix<3> *Accel_Data){
	BLA::Matrix<3> transformed_measure_h;
	BLA::Matrix<3,2> C_h_jacob;
	calc_h_Jacob(Accel_Data, &C_h_jacob, &transformed_measure_h);

	/* Compute Kalman gain K = P * C' * (R + C * P * C')^-1 */
	/* P * C'*/
	BLA::Matrix<2,3> P_times_C_trans = P_Error_Cov * (~C_h_jacob);

	/* R + C * P * C' */
	BLA::Matrix<3,3> R_plus_CPCt = R_Cov_Noise + (C_h_jacob * P_times_C_trans);

	/* inv(R + C * P * C') */
	BLA::Matrix<3,3> Inverse_Comp = BLA::Invert(R_plus_CPCt);

	BLA::Matrix<2,3> Kalman_Gain = P_times_C_trans * Inverse_Comp;

	/* Update state estimate x(n+1) = x(n) + K * (y - h) */
	X_state_estimate = X_state_estimate + Kalman_Gain * (*Accel_Data - transformed_measure_h);

	BLA::Matrix<2,2> I;

	I(0,0) = 1;
	I(0,1) = 0;
	I(1,0) = 0;
	I(1,1) = 1;
	P_Error_Cov = (I - Kalman_Gain * C_h_jacob) * P_Error_Cov;
}

void kalman_filter::calc_h_Jacob(BLA::Matrix<3> *Accel_Data, BLA::Matrix<3,2>*C_h_Jacob, BLA::Matrix<3>*transformed_measure_h){
	BLA::Matrix<3,2> &C = *C_h_Jacob;
	BLA::Matrix<3> &h = *transformed_measure_h;
	float sin_phi = sinf(X_state_estimate(0));
	float cos_phi = cosf(X_state_estimate(0));
	float sin_the = sinf(X_state_estimate(1));
	float cos_the = cosf(X_state_estimate(1));
	C(0,0) = 0.0f;
	C(0,1) = cos_the;
	C(1,0) = -cos_phi * cos_the;
	C(1,1) = sin_phi * sin_the;
	C(2,0) = sin_phi * cos_the;
	C(2,1) = cos_phi * sin_the;

	h(0) = sin_the;
	h(1) = -cos_the * sin_phi;
	h(2) = -cos_the * cos_phi;
	h *= G_MPS2;
}

float kalman_filter::get_pitch_rad(){
    return X_state_estimate(1);
}

float kalman_filter::get_roll_rad(){
    return X_state_estimate(0);
}

float kalman_filter::get_pitch_deg(){
    return get_pitch_rad() * RAD_2_DEG;
}

float kalman_filter::get_roll_deg(){
    return get_roll_rad() * RAD_2_DEG;
}
