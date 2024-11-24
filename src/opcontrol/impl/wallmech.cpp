#include "opcontrol/impl/wallmech.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace controls;

static pros::task_t task;

void wallmech::tick() {
    #ifndef MIKU_TESTENV
        const int in = robot::partner.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        const int out = robot::partner.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (in) {
            robot::wallmech.move_velocity(150);
        } else if (out) {
            robot::wallmech.move_velocity(-150);
        } else {
            robot::wallmech.move_velocity(0);
        } 
    #endif
}

void wallmech::run() {
    while (true) {
        tick();
        pros::delay(20);
    }
}

static void local_run(void* p) {
    wallmech::run();
}

void wallmech::start_task() {
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void wallmech::stop_task() {
    pros::c::task_delete(task);
}