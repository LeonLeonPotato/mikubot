#include "gui/opcontrolinfo.h"
#include "gui/utils.h"
#include "essential.h"
#include "pros/misc.hpp"

static bool initialized = false;
static pros::task_t task;

static double compute_average(std::vector<double> vec) {
    double s = 0;
    for (auto& i : vec) s += i;
    return s / vec.size();
}

static void task_func(void* args) {
    #ifndef MIKU_TESTENV
    while (true) {
            auto left_temps = robot::left_motors.get_temperature_all();
            auto right_temps = robot::right_motors.get_temperature_all();
            auto average_left_t = (int) round(compute_average(left_temps));
            auto average_right_t = (int) round(compute_average(right_temps));

            char buf_temps[64]; memset(buf_temps, 0, sizeof(buf_temps));
            snprintf(buf_temps, sizeof(buf_temps), "Temps: %d | %d", average_left_t, average_right_t);
            robot::master.set_text(1, 0, buf_temps); 
            pros::delay(150);

            auto battery = pros::battery::get_capacity();

            char buf_battery[64]; memset(buf_battery, 0, sizeof(buf_battery));
            snprintf(buf_battery, sizeof(buf_battery), "Power: %.1f PCT", battery);
            robot::master.set_text(2, 0, buf_battery);
            pros::delay(150);
    }
    #endif
}

void opcontrolinfo::init(void) {
    if (initialized) return;
    initialized = true;

    task = pros::c::task_create(task_func, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");

}

void opcontrolinfo::destroy(void) {
    if (!initialized) return;
    initialized = false;

    pros::c::task_delete(task);
}