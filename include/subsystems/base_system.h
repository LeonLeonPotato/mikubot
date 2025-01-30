#pragma once

#include "pros/apix.h"
#include "pros/rtos.h"

namespace subsystems {
class Subsystem {
    private:
        static void api_task_fn(void* args) {
            static_cast<Subsystem*>(args)->run_api();
        }

        static void task_fn(void* args) {
            static_cast<Subsystem*>(args)->run();
        }

        pros::task_t api_task = nullptr;
        pros::task_t task = nullptr;
        pros::mutex_t mutex = nullptr;

        int tick_delay = 10;

    public:
        virtual bool has_api(void) const = 0;
        virtual void api_tick(void) {}
        virtual void tick(void) = 0;

        void run(void) {
            while (true) {
                tick();
                pros::c::task_delay(tick_delay);
            }
        }

        void run_api(void) {
            while (true) {
                api_tick();
                pros::c::task_delay(tick_delay);
            }
        }

        void start_task(void) {
            if (task != nullptr) return;
            // task = pros::c::task_create(task_fn, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Subsystem Task");
        }
        void stop_task(void) {
            if (task == nullptr) return;
            pros::c::task_delete(task);
            task = nullptr;
        }
        void pause(void) {
            if (task == nullptr) return;
            pros::c::task_suspend(task);
        }
        void resume(void) {
            if (task == nullptr) return;
            pros::c::task_resume(task);
        }
        
        void start_api_task(void) {
            if (api_task != nullptr) return;
            // api_task = pros::c::task_create(api_task_fn, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Subsystem API Task");
        }
        void stop_api_task(void) {
            if (api_task == nullptr) return;
            pros::c::task_delete(api_task);
            api_task = nullptr;
        }

        void set_tick_delay(int delay) { tick_delay = delay; }
        int get_tick_delay(void) const { return tick_delay; }

        bool take_mutex(uint32_t timeout = TIMEOUT_MAX) { return pros::c::mutex_take(mutex, timeout); }
        void give_mutex(void) { pros::c::mutex_give(mutex); }
        bool poll_mutex(void) const {
            return pros::c::mutex_get_owner(mutex) == pros::c::task_get_current();
        }
        const pros::mutex_t& get_mutex(void) { return mutex; }
};
}