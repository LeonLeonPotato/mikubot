#include "autonomous/controllers.h"
#include "robot.h"

namespace controllers {
float PID::get() {
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
    float dx = desired_x - robot::x;
    float dy = desired_y - robot::y;
    float ex = dx * cos(robot::theta) + dy * sin(robot::theta);
    float ey = -dx * sin(robot::theta) + dy * cos(robot::theta);
    float etheta = robot::angular_diff(desired_theta);

    float v, w;
    if (use_vw) {
        v = desired_v;
        w = desired_w;
    } else {
        v = 0.01 * sqrt(ex * ex + ey * ey);
        w = 0.01 * etheta;
    }

    float k = 2 * zeta * sqrt(v * v + beta * w * w);

    vl = v * cos(robot::theta) + k * ex;
    vr = w + k * etheta + beta * v * sin(etheta) * ey / etheta;
}

void Ramsete::quick_ramsete(float beta, float zeta, float x, float y, float theta, float &vl, float &vr) {
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

void Ramsete::quick_ramsete(float beta, float zeta, float x, float y, float theta, float v, float w, float &vl, float &vr) {
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