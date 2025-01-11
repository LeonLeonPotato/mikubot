#include "opcontrol/impl/conveyor.h"
#include "essential.h"
#include "config.h"
#include "api.h"
#include "pros/rtos.hpp"
#include <cmath>

using namespace controls;

static pros::task_t task = nullptr;
static std::vector<float> past_hues;
static long long total_ticks = 0;
static bool color_sort_enabled = true;
static int disable_time = -1;
constexpr int disable_length = 200;

static float find_modular_average(void) {
    float cos_sum = 0; float sin_sum = 0;
    for (auto& hue : past_hues) {
        cos_sum += cosf(hue * M_PI / 180);
        sin_sum += sinf(hue * M_PI / 180);
    }
    return atan2f(
        sin_sum / past_hues.size(),
        cos_sum / past_hues.size()
    ) * 180 / M_PI;
}

static float minimum_mod_diff(float a, float b) {
    float diff = fmodf(a - b + 180, 360) - 180;
    return diff + (diff < -180) * 360;
}

void conveyor::tick() {
    robot::classifier.set_led_pwm(255);

    if (robot::master.get_digital_new_press(config::keybinds::color_sort_toggle)) {
        color_sort_enabled = !color_sort_enabled;
        robot::master.rumble(".");
    }

    const int speed = 
        (robot::master.get_digital(config::keybinds::conveyor_up)
        - robot::master.get_digital(config::keybinds::conveyor_down))
        * 600;

    const float hue = robot::classifier.get_hue();
    const bool is_ring = robot::classifier.get_proximity() > 100;
    char team = 'N';

    if (is_ring) {
        if (fabsf(minimum_mod_diff(hue, 220)) < 60) {
            printf("Blue detected | current hue: %f\n", hue);
            team = 'B';
        } else if (fabsf(minimum_mod_diff(hue, 0)) < 60) {
            printf("Red detected | current hue: %f\n", hue);
            team = 'R';
        }
        if (team != 'N' && team != robot::match::team) {
            disable_time = pros::millis() + 40;
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

    robot::conveyor.move_velocity(speed);
    total_ticks++;
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