#include "opcontrol/impl/conveyor.h"
#include "essential.h"
#include "api.h"

using namespace controls;

static pros::task_t task;

void conveyor::tick() {
    const int up = robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    const int down = robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    
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
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "conveyor");
}

void conveyor::stop_task() {
    pros::c::task_delete(task);
}