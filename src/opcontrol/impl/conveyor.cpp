#include "opcontrol/impl/conveyor.h"
#include "essential.h"
#include "config.h"
#include "api.h"
#include "mathtils.h"
#include "pros/rtos.hpp"
#include <cmath>

using namespace controls;

constexpr int disable_length = 200;

static pros::task_t api_task = nullptr;
static pros::task_t task = nullptr;

static int desired_voltage = 0;
static bool color_sort_enabled = true;
static int disable_time = -1;

static void api_task_run(void* p) {
    while (true) {
        const float hue = robot::classifier.get_hue();
        const bool is_ring = robot::classifier.get_proximity() > 100;
        char team = 'N';

        if (is_ring) {
            if (fabsf(minimum_mod_diff(hue, 220, 360)) < 60) {
                team = 'B';
            } else if (fabsf(minimum_mod_diff(hue, 0, 360)) < 60) {
                team = 'R';
            }
            if (team != 'N' && team != robot::match::team) {
                disable_time = pros::millis() + 80;
            }
        }

        if (disable_time != -1) {
            bool under = disable_time < pros::millis();
            bool over = disable_time + disable_length >= pros::millis();
            if (under && over && color_sort_enabled) {
                robot::conveyor.move_velocity(0);
                return;
            } else if (!over) {
                disable_time = -1;
            }
        }

        robot::conveyor.move_voltage(desired_voltage);

        pros::delay(10);
    }
}

void conveyor::exposed_desired_volt(float volt) {
    desired_voltage = volt;
}

void conveyor::exposed_set_color_sort(bool val) {
    color_sort_enabled = val;
}

bool conveyor::exposed_get_color_sort(void) {
    return color_sort_enabled;
}

void conveyor::start_api_task() {
    if (api_task != nullptr) return;
    api_task = pros::c::task_create(api_task_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "conveyor_api");
}

void conveyor::stop_api_task() {
    if (api_task == nullptr) return;
    pros::c::task_delete(api_task);
    api_task = nullptr;
}

void conveyor::tick() {
    start_api_task();

    if (robot::master.get_digital_new_press(config::keybinds::color_sort_toggle)) {
        color_sort_enabled = !color_sort_enabled;
        robot::master.rumble(".");
    }

    desired_voltage = 
        (robot::master.get_digital(config::keybinds::conveyor_up)
        - robot::master.get_digital(config::keybinds::conveyor_down))
        * 12000;
}

void conveyor::run() {
    while (true) {
        tick();
        pros::delay(20);
    }
}

static void local_run(void* p) {
    conveyor::run();
}

void conveyor::start_task() {
    if (task != nullptr) return;
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "conveyor");
}

void conveyor::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void conveyor::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void conveyor::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}