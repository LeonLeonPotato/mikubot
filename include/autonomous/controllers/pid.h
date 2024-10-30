#pragma once

#include <string>

namespace controllers {
class PID {
    private:
        bool registered;
        float error, last_error, integral;
        float last_time;

    public:
        float kp, ki, kd;
        float integral_limit;
        float disable_integral_limit;
        bool sign_switch_reset;

        PID() { reset(); }
        PID(
            float kp, float ki, float kd, 
            float integral_limit, float disable_integral_limit,
            bool sign_switch_reset
        ) : kp(kp), ki(ki), kd(kd), integral_limit(integral_limit), disable_integral_limit(disable_integral_limit), sign_switch_reset(sign_switch_reset) 
        {
            reset(); 
        }
        void reset(void);
        void register_error(float error);
        float get(void);
        float get(float error) {
            register_error(error);
            return get();
        }

        std::string debug(void) {
            char buffer[256];
            sprintf(buffer, "PID {last_time: %f, last_error: %f, error: %f, integral: %f}", last_time, last_error, error, integral);
            return std::string(buffer);
        }
};
} // namespace controllers