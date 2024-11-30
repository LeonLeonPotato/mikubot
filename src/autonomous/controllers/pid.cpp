#include "autonomous/controllers/pid.h"
#include "api.h"

using namespace controllers;

void PID::register_error(const float error) {
    if (!registered) {
        last_error = error;
        registered = true;
    } else {
        last_error = this->error;

        if (fabs(error) > args.disable_integral_limit || (std::signbit(error) != std::signbit(last_error) && args.sign_switch_reset)) {
            integral = 0;
        } else {
            const float integral_inc = error * (pros::micros() - last_time) / 1000000.0;
            integral = std::clamp(integral + integral_inc, -args.integral_limit, args.integral_limit);
        }
    }
    last_time = pros::micros();
    this->error = error;
}

void PID::reset() {
    error = 0;
    integral = 0;
    last_error = 0;
    registered = false;
}

float PID::get(void) {
    const float dt = (pros::micros() - last_time) / 1000000.0;
    const float derivative = (error - last_error) * (int) registered / dt;
    return args.kp * error + args.ki * integral - args.kd * derivative;
}