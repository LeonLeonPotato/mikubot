#include "opcontrol/impl/intake.h"
#include "essential.h"
#include "config.h"
#include "api.h"

using namespace controls;

static pros::task_t task = nullptr;

void intake::tick() {
    #ifndef MIKU_TESTENV
        int speed = 
            (robot::master.get_digital(config::keybinds::intake_up)
            - robot::master.get_digital(config::keybinds::intake_down))
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
    if (task != nullptr) return;
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake");
}

void intake::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void intake::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void intake::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}