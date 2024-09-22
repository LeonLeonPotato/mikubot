#include "autonomous/controllers.h"
#include <math.h>

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
}