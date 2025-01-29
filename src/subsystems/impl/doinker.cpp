#include "opcontrol/impl/doinker.h"
#include "essential.h"
#include "config.h"
#include "api.h"

using namespace controls;

static pros::task_t task = nullptr;

void doinker::tick() {
    if (robot::master.get_digital_new_press(config::keybinds::doinker)) {
        robot::doinker.toggle();
    }
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