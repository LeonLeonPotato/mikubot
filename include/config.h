#pragma once

#include "api.h" // IWYU pragma: keep

namespace config {
    using digital = pros::controller_digital_e_t;
    using analog = pros::controller_analog_e_t;

    constexpr pros::motor_brake_mode_e default_brake_mode = pros::E_MOTOR_BRAKE_COAST;
}

namespace config::keybinds {
    constexpr digital clamp = pros::E_CONTROLLER_DIGITAL_X;
    constexpr digital doinker = pros::E_CONTROLLER_DIGITAL_A;

    constexpr digital conveyor_up = pros::E_CONTROLLER_DIGITAL_L1;
    constexpr digital conveyor_down = pros::E_CONTROLLER_DIGITAL_L2;
    constexpr digital color_sort_toggle = pros::E_CONTROLLER_DIGITAL_B;

    constexpr digital intake_up = pros::E_CONTROLLER_DIGITAL_L1;
    constexpr digital intake_down = pros::E_CONTROLLER_DIGITAL_L2;

    constexpr digital wallmech_up = pros::E_CONTROLLER_DIGITAL_R1;
    constexpr digital wallmech_down = pros::E_CONTROLLER_DIGITAL_R2;
    constexpr digital wallmech_next_stage = pros::E_CONTROLLER_DIGITAL_Y;
    constexpr digital wallmech_rest = pros::E_CONTROLLER_DIGITAL_B;

};

namespace config::test {
    constexpr digital center_tick = pros::E_CONTROLLER_DIGITAL_Y;
};