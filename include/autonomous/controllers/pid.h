#pragma once

namespace controllers {
class PID {
    private:
        bool registered;
        float error, last_error, integral;

    public:
        float kp, ki, kd;
        float integral_min, integral_max;
        float disable_integral_upper, disable_integral_lower;

        PID() { reset(); }
        PID(
            float kp, float ki, float kd, 
            float integral_min, float integral_max, 
            float disable_integral_lower, float disable_integral_upper) :
            kp(kp), ki(ki), kd(kd), 
            integral_min(integral_min), integral_max(integral_max), 
            disable_integral_lower(disable_integral_lower), disable_integral_upper(disable_integral_upper
        ) {
            reset(); 
        }
        void reset(void);
        void register_error(float error);
        float get(void);

        float get(float error) {
            register_error(error);
            return get();
        }
};
} // namespace controllers