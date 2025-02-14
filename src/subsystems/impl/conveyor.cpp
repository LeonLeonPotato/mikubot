#include "subsystems/impl/conveyor.h"
#include "essential.h"
#include "config.h"
#include "api.h"
#include "mathtils.h"
#include "pros/rtos.hpp"
#include <cmath>

using namespace subsystems;

Conveyor* Conveyor::instance = nullptr;
static int disable_time = -1;

void Conveyor::api_tick(void) {
    while (true) {
        if (!robot::conveyor.poll_mutex()) robot::conveyor.acquire_mutex();

        // const float hue = robot::classifier.get_hue();
        // const bool is_ring = robot::classifier.get_proximity() > 240;
        // char team = 'N';

        // if (is_ring && hue >= 0 && hue <= 360) {
        //     if (fabsf(minimum_mod_diff(hue, 220, 360)) < 30) {
        //         team = 'B';
        //     } else if (fabsf(minimum_mod_diff(hue, 0, 360)) < 30) {
        //         team = 'R';
        //     }
        //     if (team != 'N' && team != robot::match::team) {
        //         disable_time = pros::millis() + 80;
        //     }
        // }

        // if (disable_time != -1) {
        //     bool under = pros::millis() >= disable_time;
        //     bool over = pros::millis() <= disable_time + disable_length;
        //     if (under && over && color_sort_enabled) {
        //         robot::conveyor.set_desired_voltage(0);
        //         pros::delay(20);
        //         continue;
        //     } else if (!over) {
        //         disable_time = -1;
        //     }
        // }

        robot::conveyor.set_desired_voltage(desired_voltage);

        pros::delay(20);
    }
}

void Conveyor::tick(void) {
    if (!poll_mutex()) return;

    int speed = 12000 * (
        robot::master.get_digital(config::keybinds::conveyor_up) -
        robot::master.get_digital(config::keybinds::conveyor_down)
    );

    set_desired_voltage(speed);

    if (robot::master.get_digital_new_press(config::keybinds::color_sort_toggle)) {
        toggle_color_sort_enabled();
    }
}

