#include "opcontrol/impl/doinker.h"
#include "essential.h"
#include "api.h"

using namespace controls;

static pros::task_t task;
static bool last = false;
static bool toggle = false;

void doinker::tick() {
    bool cur = robot::partner.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    if (cur == true && last == false) toggle = !toggle;

    if (toggle && !robot::doinker.is_extended()) {
        robot::doinker.extend();
    } else if (!toggle && robot::doinker.is_extended()) {
        robot::doinker.retract();
    }

    last = cur;
}

void doinker::run() {
    while (true) {
        tick();
        pros::delay(20);
    }
}

static void local_run(void* p) {
    doinker::run();
}

void doinker::start_task() {
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "conveyor");
}

void doinker::stop_task() {
    pros::c::task_delete(task);
}