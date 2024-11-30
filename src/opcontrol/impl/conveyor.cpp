#include "opcontrol/impl/conveyor.h"
#include "essential.h"
#include "config.h"
#include "api.h"

using namespace controls;

static pros::task_t task = nullptr;

void conveyor::tick() {
    #ifndef MIKU_TESTENV
        const int speed = 
            (robot::master.get_digital(config::keybinds::conveyor_up)
            - robot::master.get_digital(config::keybinds::conveyor_down))
            * 200;

        robot::conveyor.move_velocity(speed);
    #endif
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
    if (task != nullptr) return;
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "conveyor");
}

void conveyor::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void conveyor::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void conveyor::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}