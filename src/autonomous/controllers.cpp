#include "autonomous/controllers.h"
#include "robot.h"

namespace controllers {
float PID::get(void) {
    if (error > disable_integral_upper || error < disable_integral_lower) {
        integral = 0;
    } else {
        integral += error;
        integral = fmax(integral_min, fmin(integral_max, integral));
    }

    float derivative = 2*last_error - error;

    return kp * error + ki * integral + kd * derivative;
}

void Ramsete::get(float &vl, float &vr, bool use_vw) {
    if (use_vw) {
        quick_ramsete(beta, zeta, desired_x, desired_y, desired_theta, desired_v, desired_w, vl, vr);
    } else {
        quick_ramsete(beta, zeta, desired_x, desired_y, desired_theta, vl, vr);
    }
}

inline void Ramsete::quick_ramsete(float beta, float zeta, float x, float y, float theta, float &vl, float &vr) {
    float dx = x - robot::x;
    float dy = y - robot::y;
    float ex = dx * cos(robot::theta) + dy * sin(robot::theta);
    float ey = -dx * sin(robot::theta) + dy * cos(robot::theta);
    float etheta = robot::angular_diff(theta);

    float v = 0.01 * sqrt(ex * ex + ey * ey);
    float w = 0.01 * etheta;

    float k = 2 * zeta * sqrt(v * v + beta * w * w);

    vl = v * cos(robot::theta) + k * ex;
    vr = w + k * etheta + beta * v * sin(etheta) * ey / etheta;
}

inline void Ramsete::quick_ramsete(float beta, float zeta, float x, float y, float theta, float v, float w, float &vl, float &vr) {
    float dx = x - robot::x;
    float dy = y - robot::y;
    float ex = dx * cos(robot::theta) + dy * sin(robot::theta);
    float ey = -dx * sin(robot::theta) + dy * cos(robot::theta);
    float etheta = robot::angular_diff(theta);

    float k = 2 * zeta * sqrt(v * v + beta * w * w);

    vl = v * cos(robot::theta) + k * ex;
    vr = w + k * etheta + beta * v * sin(etheta) * ey / etheta;
}
} // namespace controllers