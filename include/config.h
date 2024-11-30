#pragma once

#include "api.h"

namespace config {
    constexpr pros::motor_brake_mode_e default_brake_mode = pros::E_MOTOR_BRAKE_COAST;
}

namespace config::keybinds {
    using digital = pros::controller_digital_e_t;
    using analog = pros::controller_analog_e_t;

    constexpr digital clamp = pros::E_CONTROLLER_DIGITAL_X;
    constexpr digital doinker = pros::E_CONTROLLER_DIGITAL_A;

    constexpr digital conveyor_up = pros::E_CONTROLLER_DIGITAL_L1;
    constexpr digital conveyor_down = pros::E_CONTROLLER_DIGITAL_R1;

    constexpr digital intake_up = pros::E_CONTROLLER_DIGITAL_L1;
    constexpr digital intake_down = pros::E_CONTROLLER_DIGITAL_R1;

    constexpr digital wallmech_up = pros::E_CONTROLLER_DIGITAL_X;
    constexpr digital wallmech_down = pros::E_CONTROLLER_DIGITAL_B;
};