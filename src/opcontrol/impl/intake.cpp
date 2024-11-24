#include "opcontrol/impl/intake.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace controls;

static pros::task_t task;

void intake::tick() {
    #ifndef MIKU_TESTENV
        int speed = 
            (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)
            - robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            * 200;

        robot::intake.move_velocity(speed);
    #endif
}

void intake::run() {
    while (true) {
        tick();
        pros::delay(20);
    }
}

static void local_run(void* p) {
    intake::run();
}

void intake::start_task() {
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void intake::stop_task() {
    pros::c::task_delete(task);
}