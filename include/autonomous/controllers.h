#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

namespace controllers {
class PID {
    private:
        bool registered;
        float kp, ki, kd;
        float error, last_error, integral;
        float integral_min, integral_max;
        float disable_integral_upper, disable_integral_lower;

    public:
        PID(float kp, float ki, float kd, float integral_min, float integral_max, float disable_integral_lower, float disable_integral_upper):
            kp(kp), ki(ki), kd(kd), integral_min(integral_min), integral_max(integral_max), disable_integral_lower(disable_integral_lower), disable_integral_upper(disable_integral_upper) {
            error = 0;
            integral = 0;
            last_error = 0;
            registered = false;
        }
        void reset(void) {
            registered = false;
            error = 0;
            integral = 0;
            last_error = 0;
            registered = false;
        }
        void register_error(float error) {
            if (!registered) {
                last_error = error;
                registered = true;
            } else {
                last_error = this->error;
            }
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

class KalmanFilter {
    private:
        float x, p, q, r;
    
    public:
        KalmanFilter(float x, float p, float q, float r):
            x(x), p(p), q(q), r(r) {}
        void update(float z) {
            float k = p / (p + r);
            x = x + k * (z - x);
            p = (1 - k) * p + q;
        }
        float get(void) {
            return x;
        }
};
} // namespace controllers