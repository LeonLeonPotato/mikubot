#include "opcontrol/impl/ejector.h"
#include "autonomous/strategies.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace controls;

static pros::task_t task;
static long long last_detection = -1;
static char color = 'N';
static const double start_t = 0.05;
static int iters = 0;
static double default_hue = -1;

void ejector::tick() {
    if (default_hue == -1) {
        default_hue = robot::classifier.get_hue();
    }

    auto hue = robot::classifier.get_hue();
    iters++;
    auto diff = fmod(hue - default_hue + 180, 360) - 180;

    if (diff < -30) {
        last_detection = pros::micros();
        color = 'R';
    } else if (diff > 50) {
        last_detection = pros::micros();
        color = 'B';
    }

    // if (iters % 30 == 0)
    //     printf("Hue: %f | Last detection: %lld | color: %c\n", diff, last_detection, color);

    if (color != strategies::config::team && color != 'N' && last_detection != -1) {
        auto dt = (pros::micros() - last_detection) / 1000000.0f;
        if (dt > start_t && !robot::ejector.is_extended()) {
            robot::ejector.extend();
        } else if (dt > start_t + 0.2) {
            last_detection = -1;
            color = 'N';
            robot::ejector.retract();
        }
    }
}

void ejector::run() {
    while (true) {
        tick();
        pros::delay(5);
    }
}

static void local_run(void* p) {
    ejector::run();
}

void ejector::start_task() {
    task = pros::c::task_create(local_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void ejector::stop_task() {
    pros::c::task_delete(task);
}