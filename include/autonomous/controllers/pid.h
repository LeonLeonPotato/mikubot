#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

namespace controllers {
    class PID {
    private:
        bool registered;
        float error, last_error, integral;

    public:
        float kp, ki, kd;
        float integral_min, integral_max;
        float disable_integral_upper, disable_integral_lower;

        PID();
        PID(float kp, float ki, float kd, float integral_min, float integral_max, float disable_integral_lower, float disable_integral_upper):
            kp(kp), ki(ki), kd(kd), integral_min(integral_min), integral_max(integral_max), disable_integral_lower(disable_integral_lower), disable_integral_upper(disable_integral_upper) 
        {
            PID();
        }
        void reset(void);
        void register_error(float error);
        float get(void);
};

inline PID::PID() {
    error = 0;
    integral = 0;
    last_error = 0;
    registered = false;
}

inline void PID::reset() {
    error = 0;
    integral = 0;
    last_error = 0;
    registered = false;
}

inline float PID::get(void) {
    float derivative = (error - last_error) * (int) registered;
    return kp * error + ki * integral - kd * derivative;
}
}