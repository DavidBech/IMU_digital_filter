#ifndef KALMAN_FILTER_HPP_DB_
#define KALMAN_FILTER_HPP_DB_

class kalman_filter{
    public:
    kalman_filter();

    void new_measure(float accel[3], float gyro[3]);

    float get_pitch_rad();
    float get_roll_rad();
    float get_pitch_deg();
    float get_roll_deg();

    private:

    void predict(float p_rps, float q_rps, float r_rps);
    void update(float ax_mps2, float ay_mps2, float az_mps2);

    unsigned measure_count = 0;
    unsigned predicts_per_update = 10;

	float phi_r;
	float theta_r;

    // Covariance Matrix
    float P[2][2];

    // Process and measurement noise
    float Q[2];
    float R[3];

    float sampleTime_s;
};

#endif //KALMAN_FILTER_HPP_DB_