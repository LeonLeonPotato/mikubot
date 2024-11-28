#pragma once

#include <string>

namespace controllers {
struct PIDArgs {
    float kp, ki, kd;
    float integral_limit = infinityf();
    float disable_integral_limit = infinityf();
    bool sign_switch_reset;
};

class PID {
    private:
        bool registered;
        float error, last_error, integral;
        float last_time;

    public:
        const PIDArgs& args;

        PID() : args(PIDArgs()) { reset(); }
        PID(const PIDArgs& args) : args(args) { reset(); };

        void reset(void);

        void register_error(const float error);

        float get(void);
        float get(const float error) {
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