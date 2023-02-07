#include <BasicLinearAlgebra.h>
#include <math.h>

#include "IMU_CONST.hpp"
#include "kalman_filter.hpp"

kalman_filter::kalman_filter(){

	F_state_trans(0, 0) = 1;
	F_state_trans(1, 0) = 0;

	P_Error_Cov(0,0) = 0.0f;
	P_Error_Cov(0,1) = 0.0f;
	P_Error_Cov(1,0) = 0.0f;
	P_Error_Cov(0,1) = 0.0f;

	Q_Cov_Noise(0,0) = 0.01f;
	Q_Cov_Noise(0,1) = 0.00f;
	Q_Cov_Noise(1,0) = 0.01f;
	Q_Cov_Noise(0,1) = 0.00f;

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
	A(0,0) = (GD(1)*cos_phi - GD(3)*sin_phi) * tan_the; // d phi / d phi
	A(0,1) = (GD(1)*sin_phi	+ GD(3)*cos_phi) * sec_the * sec_the; // d phi / d theta
	A(1,0) = -GD(1)*sin_phi - GD(2)*cos_phi; // d theta / d phi
	A(1,1) = 0.0f; // d theta / d theta
}

void kalman_filter::predict_update_error_covariance(BLA::Matrix<3> *Gyro_Data){
	// P(n+1) = P(n) + t(A P(n) + P(n) ~A + Q)
	// A is Jacobian of F, ~A is transposition of A
	BLA::Matrix<2,2> A_Jacob_F;
	calc_Jacobian_of_F(Gyro_Data, &A_Jacob_F);

	P_Error_Cov = P_Error_Cov + (A_Jacob_F * P_Error_Cov + P_Error_Cov * ~A_Jacob_F + Q_Cov_Noise) * sampleTime_s;
}

void kalman_filter::update(BLA::Matrix<3> *Accel_Data){
	/* Normalise accelerometer readings */
	float accNormFactor = 1.0f / sqrtf(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);

	float ax_norm 		= ax_mps2 * accNormFactor;
	float ay_norm 		= ay_mps2 * accNormFactor;
	float az_norm 		= az_mps2 * accNormFactor;

	/* Compute Jacobian of output function C(x,u) = dh(x,u)/dx */
	float sp = sinf(phi_r);
	float cp = cosf(phi_r);
	float st = sinf(theta_r);
	float ct = cosf(theta_r);


	float C[3][2] = {	{0.0f, 		ct},
						{-cp * ct, 	sp * st},
						{sp * ct, 	cp * st} };

	/* Compute Kalman gain K = P * C' * (R + C * P * C ')^-1 in steps (note that C[0][0] = 0!) */

	/* P * C'*/
	float PCt[2][3] = { {P[0][1] * C[0][1], P[0][0] * C[1][0] + P[0][1] * C[1][1], P[0][0] * C[2][0] + P[0][1] * C[2][1]},
						{P[1][1] * C[0][1], P[1][0] * C[1][0] + P[1][1] * C[1][1], P[1][0] * C[2][0] + P[1][1] * C[2][1]} };

	/* R + C * P * C' */
	float RCPCt[3][3] = { 	{C[0][1] * PCt[1][0] + R[0],			C[0][1] * PCt[1][1], 									C[0][1] * PCt[1][2]},
							{C[1][0] * PCt[0][0] + C[1][1] * PCt[1][0],	C[1][0] * PCt[0][1] + C[1][1] * PCt[1][1] + R[1],	C[1][0] * PCt[0][2] + C[1][1] * PCt[1][2]},
							{C[2][0] * PCt[0][0] + C[2][1] * PCt[1][0],	C[2][0] * PCt[0][1] + C[2][1] * PCt[1][1],				C[2][0] * PCt[0][2] + C[2][1] * PCt[1][2] + R[2]} };

	/* inv(R + C * P * C') */
	float detMatInv = 1.0f / (RCPCt[0][0] * (RCPCt[2][2] * RCPCt[1][1] - RCPCt[2][1] * RCPCt[1][2])
					- RCPCt[1][0] * (RCPCt[2][2] * RCPCt[0][1] - RCPCt[2][1] * RCPCt[0][2])
					+ RCPCt[2][0] * (RCPCt[1][2] * RCPCt[0][1] - RCPCt[1][1] * RCPCt[0][2]));

	float matInv[3][3] = { 	{	RCPCt[2][2] * RCPCt[1][1] - RCPCt[2][1] * RCPCt[1][2], -(	RCPCt[2][2] * RCPCt[0][1] - RCPCt[2][1] * RCPCt[0][2]), 	RCPCt[1][2] * RCPCt[0][1] - RCPCt[1][1] * RCPCt[0][2] },
							{-(	RCPCt[2][2] * RCPCt[1][0] - RCPCt[2][0] * RCPCt[1][2]), 	RCPCt[2][2] * RCPCt[0][0] - RCPCt[2][0] * RCPCt[0][2], -(	RCPCt[1][2] * RCPCt[0][0] - RCPCt[1][0] * RCPCt[0][2]) },
							{	RCPCt[2][1] * RCPCt[1][0] - RCPCt[2][0] * RCPCt[1][1], -(	RCPCt[2][1] * RCPCt[0][0] - RCPCt[2][0] * RCPCt[0][1]), 	RCPCt[1][1] * RCPCt[0][0] - RCPCt[1][0] * RCPCt[0][1]} };

	for (unsigned int i = 0; i < 3; i++) {

		for (unsigned int j = 0; j < 3; j++) {

			matInv[i][j] *= detMatInv;

		}

	}

	/* C' * inv(R + C * P * C') */
	float CtmatInv[2][3] = { 	{						  C[1][0] * matInv[1][0] + C[2][0] * matInv[2][0], 							C[1][0] * matInv[1][1] + C[2][0] * matInv[2][1], 						  C[1][0] * matInv[1][2] + C[2][0] * matInv[2][2]},
								{C[0][1] * matInv[0][0] + C[1][1] * matInv[1][0] + C[2][1] * matInv[2][0], C[0][1] * matInv[0][1] + C[1][1] * matInv[1][1] + C[2][1] * matInv[2][1], C[0][1] * matInv[0][2] + C[1][1] * matInv[1][2] + C[2][1] * matInv[2][2]} };

	/* K = P * C' * inv(R + C * P * C') */
	float K[2][3] = { 	{P[0][0] * CtmatInv[0][0] + P[0][1] * CtmatInv[1][0], P[0][0] * CtmatInv[0][1] + P[0][1] * CtmatInv[1][1], P[0][0] * CtmatInv[0][2] + P[0][1] * CtmatInv[1][2]},
						{P[1][0] * CtmatInv[0][0] + P[1][1] * CtmatInv[1][0], P[1][0] * CtmatInv[0][1] + P[1][1] * CtmatInv[1][1], P[1][0] * CtmatInv[0][2] + P[1][1] * CtmatInv[1][2]} };

	/* Update state covariance matrix P(n+1) = (I - K * C) * P(n) */
	float IminKC[2][2] = { 	{1.0f - (K[0][1] * C[1][0] + K[1][0] * C[2][0]), -(K[0][1] * C[1][1] + K[1][0] * C[2][1])},
							{-(K[1][1] * C[1][0] + K[1][2] * C[2][0]), 1.0f - (K[1][1] * C[1][1] + K[1][2] * C[2][1])} };

	float Pnew[2][2] = { 	{IminKC[0][0] * P[0][0] + IminKC[0][1] * P[1][0], IminKC[0][0] * P[0][1] + IminKC[0][1] * P[1][1]},
							{IminKC[1][0] * P[0][0] + IminKC[1][1] * P[1][0], IminKC[1][0] * P[0][1] + IminKC[1][1] * P[1][1]} };

	P[0][0] = Pnew[0][0]; P[0][1] = Pnew[0][1];
	P[1][0] = Pnew[1][0]; P[1][1] = Pnew[1][1];

	/* Compute output function h(x,u) */
	float h[3] = {	 sinf(theta_r),
					-cosf(theta_r) * sinf(phi_r),
					-cosf(theta_r) * cosf(phi_r) };

	/* Update state estimate x(n+1) = x(n) + K * (y - h) */
	phi_r 		= K[0][0] * (ax_norm - h[0]) + K[0][1] * (ay_norm - h[1]) + K[0][2] * (az_norm - h[2]);
	theta_r 	= K[1][0] * (ax_norm - h[0]) + K[1][1] * (ay_norm - h[1]) + K[1][2] * (az_norm - h[2]);
}


float kalman_filter::get_pitch_rad(){
    return X_state_estimate(2);
}

float kalman_filter::get_roll_rad(){
    return X_state_estimate(1);
}

float kalman_filter::get_pitch_deg(){
    return get_pitch_rad() * RAD_2_DEG;
}

float kalman_filter::get_roll_deg(){
    return get_roll_rad() * RAD_2_DEG;
}

void kalman_filter::update_correct_predictions(){

}

void kalman_filter::update_correct_error_covariance(){

}
