#include "opcontrol/impl/wallmech.h"
#include "autonomous/controllers/pid.h"
#include "essential.h"
#include "config.h"
#include "api.h"
#include "pros/rtos.hpp"

using namespace controls;
using State = wallmech::State;

static pros::task_t task = nullptr;
static pros::task_t api_task = nullptr;
static controllers::PID pid(0.02, 0.01, 0);

float positions[3] = {0.0f, 45.9f, 175.9f};
static int special_fire_thing = -1;

static State set_state = State::RESTING;

static void api_task_func(void* p) {
    while (true) {
        pros::delay(10);
        if (set_state == State::OVERRIDE) {
            continue;
        }

        if (special_fire_thing != -1) {
            if (pros::millis() < special_fire_thing) {
                robot::conveyor.move_voltage(-6000);
            } else {
                special_fire_thing = -1;
                robot::conveyor.move_voltage(0);
            }
        }

        const auto& desired = positions[(int) set_state];
        const auto current = robot::wallmech_encoder.get_position() / 100.0f;

        if (std::abs(desired - current) < 2.5f) {
            robot::wallmech.brake();
        } else {
            float control = pid.get(desired - current);
            control = std::clamp(control, -1.0f, 1.0f);
            robot::wallmech.move_voltage((int) roundf(control * 12000));
        }
    }
}

void wallmech::exposed_go_to_state(State state) {
    set_state = state;

    if (set_state == State::FIRING) {
        special_fire_thing = pros::millis() + 100;
    }
}

void wallmech::start_api_task() {
    if (api_task == nullptr) {
        api_task = pros::c::task_create(api_task_func, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "wallmech_api");
    }
}

void wallmech::stop_api_task() {
    if (api_task == nullptr) return;
    pros::c::task_delete(api_task);
    api_task = nullptr;
}

void wallmech::tick() {
    start_api_task();

    bool next_stage = robot::master.get_digital_new_press(config::keybinds::wallmech_next_stage);
    bool rest = robot::master.get_digital_new_press(config::keybinds::wallmech_rest);

    if (robot::master.get_digital(config::keybinds::wallmech_up) 
        || robot::master.get_digital(config::keybinds::wallmech_down)) {
        set_state = State::OVERRIDE;
    } else if (set_state == State::OVERRIDE && (next_stage || rest)) 
    {
        set_state = State::RESTING;
    }

    switch (set_state) {
        case State::RESTING: {
            if (next_stage) {
                set_state = State::PRIMED;
            }
            break;
        }
        case State::PRIMED: {
            if (rest) {
                set_state = State::RESTING;
            } else if (next_stage) {
                set_state = State::FIRING;
                special_fire_thing = pros::millis() + 100;
            }
            break;
        }
        case State::FIRING: {
            if (rest) {
                set_state = State::RESTING;
            } else if (next_stage) {
                set_state = State::PRIMED;
            }
            break;
        }
        case State::OVERRIDE: {
            const int speed = (robot::master.get_digital(config::keybinds::wallmech_up) 
                - robot::master.get_digital(config::keybinds::wallmech_down)) 
                * 12000;

            if (speed == 0) robot::wallmech.brake();
            else robot::wallmech.move_voltage(speed);
            break;
        }
    }
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