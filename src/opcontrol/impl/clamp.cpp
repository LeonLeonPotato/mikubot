#include "opcontrol/impl/clamp.h"
#include "essential.h"
#include "config.h"

using namespace controls;

static pros::task_t task = nullptr;
static bool last = false;
static bool toggle = false;

void clamp::tick() {
    #ifndef MIKU_TESTENV
        const bool cur = robot::master.get_digital(config::keybinds::clamp);

        if (cur && !last) toggle = !toggle;

        if (toggle && !robot::clamp.is_extended()) {
            robot::clamp.extend();
        } else if (!toggle && robot::clamp.is_extended()) {
            robot::clamp.retract();
        }

        last = cur;
    #endif
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
    if (task != nullptr) return;
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "clamp");
}

void clamp::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void clamp::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void clamp::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}