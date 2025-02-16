#include "autonomous/controllers/pid.h"
#include "api.h"
#include <cmath>

using namespace controllers;

void PID::register_error(const float error) {
    if (isnan(error)) return;

    last_error = this->error;
    this->error = error;
    last_time = cur_time;
    cur_time = pros::micros();

    if (!registered) {
        last_error = error;
        last_time = cur_time - 1;
        registered = true;
    } else {
        if (fabs(error) > args.disable_integral_limit || (args.sign_switch_reset && error * last_error < 0)) {
            integral = 0;
        } else {
            float integral_inc = error * (cur_time - last_time) / 1000000.0;
            if (isnan(integral_inc)) integral_inc = 0;
            integral = std::clamp(integral + integral_inc, -args.integral_limit, args.integral_limit);
        }
    }
}

void PID::reset() {
    registered = false;
    error = 0;
    last_error = 0;
    integral = 0;
    last_time = 0;
    cur_time = 1;
}

float PID::get(void) const {
    const float dt = (cur_time - last_time) / 1000000.0f;
    float derivative = (error - last_error) * (int) registered / dt;
    if (isnan(derivative)) derivative = 0;
    float I = args.ki * integral;
    if (isnan(I)) I = 0;
    return args.kp * error + I + args.kd * derivative;
}