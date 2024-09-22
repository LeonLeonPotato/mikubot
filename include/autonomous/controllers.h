#ifndef _MIKUBOT_AUTONOMOUS_CONTROLLERS_H_
#define _MIKUBOT_AUTONOMOUS_CONTROLLERS_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

namespace controllers {
class PID {
    private:
        float kp, ki, kd;
        float error, last_error, integral;
        float integral_min, integral_max;
        float disable_integral_upper, disable_integral_lower;

    public:
        PID(float kp, float ki, float kd, float integral_min, float integral_max, float disable_integral_upper, float disable_integral_lower):
            kp(kp), ki(ki), kd(kd), integral_min(integral_min), integral_max(integral_max), disable_integral_upper(disable_integral_upper), disable_integral_lower(disable_integral_lower) {
            error = 0;
            integral = 0;
            last_error = 0;
        }
        void reset(void) {
            error = 0;
            integral = 0;
            last_error = 0;
        }
        void register_error(float error) {
            this->last_error = this->error;
            this->error = error;
        }
        float get();
};

class Ramsete {
    // stub
};
} // namespace controllers

#endif