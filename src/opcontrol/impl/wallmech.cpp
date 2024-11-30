#include "opcontrol/impl/wallmech.h"
#include "essential.h"
#include "config.h"
#include "api.h"

using namespace controls;

static pros::task_t task = nullptr;

void wallmech::tick() {
    #ifndef MIKU_TESTENV
        const int speed = 
            (robot::partner.get_digital(config::keybinds::wallmech_up)
            - robot::partner.get_digital(config::keybinds::wallmech_down))
            * 200;

        robot::wallmech.move_velocity(speed);
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
    if (task != nullptr) return;
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "wallmech");
}

void wallmech::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void wallmech::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void wallmech::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}