#include "opcontrol/impl/clamp.h"
#include "essential.h"
#include "api.h"

using namespace controls;

static pros::task_t task;
static bool last = false;
static bool toggle = false;

void clamp::tick() {
    bool cur = robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    if (cur == true && last == false) toggle = !toggle;

    if (toggle && !robot::clamp.is_extended()) {
        robot::clamp.extend();
    } else if (!toggle && robot::clamp.is_extended()) {
        robot::clamp.retract();
    }

    last = cur;
}

void clamp::run() {
    while (true) {
        tick();
        pros::delay(20);
    }
}

static void local_run(void* p) {
    clamp::run();
}

void clamp::start_task() {
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "clamp");
}

void clamp::stop_task() {
    pros::c::task_delete(task);
}