#pragma once

#include "pros/rtos.h"
#include "subsystems/impl/driving.h" // IWYU pragma: export
#include "subsystems/impl/conveyor.h" // IWYU pragma: export
#include "subsystems/impl/clamp.h" // IWYU pragma: export
#include "subsystems/impl/wallmech.h" // IWYU pragma: export
#include "subsystems/impl/doinker.h" // IWYU pragma: export

#include <vector>
#include <functional>

namespace subsystems {
class Subsystem {
    private:
        pros::task_t api_task = nullptr;
        pros::task_t task = nullptr;
        int tick_delay = 10;

    protected:
        virtual void tick(void) = 0;

        virtual void start_task(void) {
            if (task != nullptr) return;
            task = pros::c::task_create([] (void* args) {
                static_cast<Subsystem*>(args)->tick();
            }, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "subsystem");
        }
        virtual void stop_task(void) = 0;
        virtual void pause(void) = 0;
}