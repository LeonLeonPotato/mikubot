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
    static int cnt = 0;
    static int unstuck_time = -1;

    while (true) {
        if (!robot::conveyor.poll_mutex()) robot::conveyor.acquire_mutex();

        if (override_mode) {
            robot::conveyor.set_desired_voltage(override_voltage);
            pros::delay(10);
            continue;
        }
        
        const float hue = robot::classifier.get_hue();
        const bool is_ring = robot::classifier.get_proximity() > 240;
        char team = 'N';

        if (is_ring && hue >= 0 && hue <= 360) {
            if (fabsf(minimum_mod_diff(hue, 220, 360)) < 30) {
                team = 'B';
            } else if (fabsf(minimum_mod_diff(hue, 0, 360)) < 30) {
                team = 'R';
            }
            if (team != 'N' && team != robot::match::team) {
                disable_time = pros::millis() + 5;
            }
        }

        if (disable_time != -1) {
            bool under = pros::millis() >= disable_time;
            bool over = pros::millis() <= disable_time + disable_length;
            if (under && over && color_sort_enabled) {
                robot::conveyor.set_desired_voltage(-12000);
                pros::delay(5);
                continue;
            } else if (!over) {
                disable_time = -1;
            }
        }

        // if (robot::conveyor.get_efficiency_average() < 10 && abs(desired_voltage) > 6000) {
        //     cnt++;
        // } else cnt = 0;

        // if (cnt > 100) {
        //     unstuck_time = pros::millis();
        //     cnt = 0;
        // }

        // if (unstuck_time != -1) {
        //     if (pros::millis() - unstuck_time < 100)
        //         robot::conveyor.set_desired_voltage(-6000);
        //     else
        // } else {
            robot::conveyor.set_desired_voltage(desired_voltage);
        // }

        pros::delay(10);
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

