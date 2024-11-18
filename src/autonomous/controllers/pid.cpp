#include "autonomous/controllers/pid.h"
#include "api.h"

using namespace controllers;

void PID::register_error(float error) {
    this->last_time = this->cur_time;
    this->last_error = this->error;
    this->cur_time = pros::micros();
    this->error = error;

    if (fabs(error) > disable_integral_limit || (std::signbit(error) != std::signbit(last_error) && sign_switch_reset)) {
        integral = 0;
    } else {
        float integral_inc = error * (pros::micros() - last_time) / 1000000.0;
        integral = std::clamp(integral + integral_inc, -integral_limit, integral_limit);
    }
}

void PID::reset() {
    error = 0; last_error = 0;
    integral = 0;
    cur_time = -1; last_time = -1;
}

float PID::get(void) {
    float dt = (cur_time - last_time) / 1000000.0;
    float derivative = (error - last_error) / dt * (int) (last_time != -1);
    return kp * error + ki * integral - kd * derivative;
}