#include "autonomous/controllers/pid.h"
#include "api.h"

using namespace controllers;

void PID::register_error(float error) {
    if (!registered) {
        last_error = error;
        registered = true;
    } else {
        last_error = this->error;

        if (fabs(error) > disable_integral_limit || (std::signbit(error) != std::signbit(last_error) && sign_switch_reset)) {
            integral = 0;
        } else {
            float integral_inc = error * (pros::micros() - last_time) / 1000000.0;
            integral = std::clamp(integral + integral_inc, -integral_limit, integral_limit);
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
    float dt = (pros::micros() - last_time) / 1000000.0;
    float derivative = (error - last_error) * (int) registered / dt;
    return kp * error + ki * integral - kd * derivative;
}