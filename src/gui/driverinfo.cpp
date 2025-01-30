#include "gui/driverinfo.h"
#include "gui/utils.h"  // IWYU pragma: keep
#include "essential.h"
#include "pros/misc.hpp"

static pros::task_t task = nullptr;

static void task_func(void* args) {
    while (true) {
        float left_temps = robot::left_motors.get_temperature_average();
        float right_temps = robot::right_motors.get_temperature_average();

        char buf_temps[64] = {0};
        snprintf(buf_temps, sizeof(buf_temps), "Temps: %d | %d", (int) round(left_temps), (int) round(right_temps));
        robot::master.set_text(1, 0, buf_temps); 
        pros::delay(110);

        auto battery = pros::battery::get_capacity();

        char buf_battery[64] = {0};
        snprintf(buf_battery, sizeof(buf_battery), "Power: %.1f PCT", battery);
        robot::master.set_text(2, 0, buf_battery);
        pros::delay(110);
    }
}

void driverinfo::init(void) {
    if (task != nullptr) return;
    // task = pros::c::task_create(task_func, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void driverinfo::destroy(void) {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}