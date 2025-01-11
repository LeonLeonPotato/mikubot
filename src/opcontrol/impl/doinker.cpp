#include "opcontrol/impl/doinker.h"
#include "essential.h"
#include "config.h"
#include "api.h"

using namespace controls;

static pros::task_t task = nullptr;
static bool last = false;
static bool toggle = false;

void doinker::tick() {
    const bool cur = robot::master.get_digital(config::keybinds::doinker);

    if (cur && !last) toggle = !toggle;

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
    if (task != nullptr) return;
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "conveyor");
}

void doinker::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void doinker::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void doinker::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}