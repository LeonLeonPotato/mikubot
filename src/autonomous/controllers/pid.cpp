#include "autonomous/controllers/pid.h"
#include <cmath>

using namespace controllers;

void PID::register_error(float error) {
    if (!registered) {
        last_error = error;
        registered = true;
    } else {
        last_error = this->error;

        if (error > disable_integral_upper || error < disable_integral_lower || (std::signbit(error) != std::signbit(last_error))) {
            integral = 0;
        } else {
            integral += error;
            integral = fmax(integral_min, fmin(integral_max, integral));
        }
    }
    this->error = error;
}

void PID::reset() {
    error = 0;
    integral = 0;
    last_error = 0;
    registered = false;
}

#include <iostream>
float PID::get(void) {
    float derivative = (error - last_error) * (int) registered;
    // printf("Error: %f, Integral: %f, Derivative: %f\n", error, integral, derivative);
    return kp * error + ki * integral - kd * derivative;
}