
#ifndef COMPLEMENTARY_FILTER_HPP_DB_
#define COMPLEMENTARY_FILTER_HPP_DB_

class complementary_filter{
    public:
    complementary_filter();

    void set_period_ms(unsigned period_ms);
    unsigned get_period_ms();


    void set_alpha(float alpha);
    float get_alpha();
    float get_pitch_rad();
    float get_roll_rad();
    float get_pitch_deg();
    float get_roll_deg();
    void new_measurement(float acceleration[3], float gyro[3]);


    private:
    float pitch_rad;
    float roll_rad;
    float alpha;
    unsigned measure_period_ms;
};

#endif //COMPLEMENTARY_FILTER_HPP_DB_
