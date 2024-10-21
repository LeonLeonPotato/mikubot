#include "autonomous/controllers/ramsete.h"
#include "essential.h"

using namespace controllers;

std::pair<float, float> Ramsete::get(bool use_vw) {
    if (use_vw) {
        return quick_ramsete(beta, zeta, desired_x, desired_y, desired_theta, desired_v, desired_w);
    } else {
        quick_ramsete(beta, zeta, desired_x, desired_y, desired_theta);
    }
}

inline std::pair<float, float> Ramsete::quick_ramsete(float beta, float zeta, float x, float y, float theta) {
    float dx = x - robot::x;
    float dy = y - robot::y;
    float ex = dx * cos(robot::theta) + dy * sin(robot::theta);
    float ey = -dx * sin(robot::theta) + dy * cos(robot::theta);
    float etheta = robot::angular_diff(theta);

    float v = 0.01 * sqrt(ex * ex + ey * ey);
    float w = 0.01 * etheta;

    float k = 2 * zeta * sqrt(v * v + beta * w * w);

    float vl = v * cos(robot::theta) + k * ex;
    float vr = w + k * etheta + beta * v * sin(etheta) * ey / etheta;

    return {vl, vr};
}

inline std::pair<float, float> Ramsete::quick_ramsete(float beta, float zeta, float x, float y, float theta, float v, float w) {
    float dx = x - robot::x;
    float dy = y - robot::y;
    float ex = dx * cos(robot::theta) + dy * sin(robot::theta);
    float ey = -dx * sin(robot::theta) + dy * cos(robot::theta);
    float etheta = robot::angular_diff(theta);

    float k = 2 * zeta * sqrt(v * v + beta * w * w);

    float vl = v * cos(robot::theta) + k * ex;
    float vr = w + k * etheta + beta * v * sin(etheta) * ey / etheta;

    return {vl, vr};
}