#ifndef KALMAN_FILTER_HPP_DB_
#define KALMAN_FILTER_HPP_DB_

#include <BasicLinearAlgebra.h>

class kalman_filter{
    public:
    kalman_filter();

    void new_measure(float accel[3], float gyro[3]);

    float get_pitch_rad();
    float get_roll_rad();
    float get_pitch_deg();
    float get_roll_deg();

    private:

    void predict(BLA::Matrix<3>*);
    void predict_update_state(BLA::Matrix<3>*);
    void predict_update_error_covariance(BLA::Matrix<3>*);
    void update(BLA::Matrix<3>*);
    void update_correct_predictions();
    void update_correct_error_covariance();

    void calc_F_state_trans();
    void calc_Jacobian_of_F(BLA::Matrix<3> *Gyro_Data, BLA::Matrix<2,2>*A);

    unsigned measure_count = 0;
    unsigned predicts_per_update = 10;

	/* 1   sin(phi)*tan(theta) cos(phi)*tan(theta)
	   0   cos(phi)             -sin(phi)*/
	BLA::Matrix<2, 3> F_state_trans;

	/* (1)=phi/roll 
	   (2)=theta/pitch */
	BLA::Matrix<2> X_state_estimate;

    // Covariance Matrix
	BLA::Matrix<2, 2> P_Error_Cov;

    // Process and measurement noise
	BLA::Matrix<2, 2> Q_Cov_Noise; 
	BLA::Matrix<3, 3> R_Cov_Noise;

    float sampleTime_s;
};

#endif //KALMAN_FILTER_HPP_DB_