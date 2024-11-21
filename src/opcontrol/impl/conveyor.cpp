#include "opcontrol/impl/conveyor.h"
#include "essential.h"
#include "api.h"

using namespace controls;

static pros::task_t task;
int destuck_ticks = 0;
int destuck_start_time = -1;

void conveyor::tick() {
    int speed = robot::partner.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    speed = (int) ((speed / 127.0f) * 200.0f);

    double eff = robot::conveyor.get_efficiency();
        
    if (abs(speed) > 3) {
        if (destuck_start_time == -1) {
            if (eff < 10) {
                destuck_ticks++;
                if (destuck_ticks >= 10) {
                    destuck_start_time = pros::millis();
                }
            } else {
                destuck_ticks = 0;
            }
        } else if (pros::millis() - destuck_start_time > 2000) {
            destuck_start_time = -1;
            destuck_ticks = 0;
        }
    }

    if (destuck_start_time == -1) {
        robot::conveyor.move_velocity(speed);
    } else {
        robot::conveyor.move_velocity(-speed);
    }
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