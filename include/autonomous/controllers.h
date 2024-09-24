#pragma once

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
        float get(void);
};

class Ramsete {
    public:
        float beta, zeta;
        float desired_x, desired_y, desired_theta;
        float desired_v, desired_w;
    
        Ramsete(float beta, float zeta):
            beta(beta), zeta(zeta) {
            desired_x = 0;
            desired_y = 0;
            desired_theta = 0;
        }

        void get(float &vl, float &vr, bool use_vw = false);

    static void quick_ramsete(float beta, float zeta, float x, float y, float theta, float v, float w, float &vl, float &vr);
    static void quick_ramsete(float beta, float zeta, float x, float y, float theta, float &vl, float &vr);
};
} // namespace controllers