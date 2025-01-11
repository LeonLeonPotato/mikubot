#include "opcontrol/impl/wallmech.h"
#include "autonomous/controllers/pid.h"
#include "essential.h"
#include "config.h"
#include "api.h"
#include "pros/rtos.hpp"

using namespace controls;

static pros::task_t task = nullptr;
static controllers::PID pid(1.0, 0.0, 0.1);

float positions[3] = {0.0f, 600.0f, 2000.0f};
enum class State {
    RESTING = 0,
    PRIMED = 1,
    FIRING = 2,
    OVERRIDE = 3
};
static State set_state = State::RESTING;
static int special_fire_thing = -1;

bool last_next_stage = false;
bool last_rest = false;

void wallmech::tick() {
    const bool next_stage = robot::master.get_digital(config::keybinds::wallmech_next_stage);
    const bool rest = robot::master.get_digital(config::keybinds::wallmech_rest);

    if (robot::master.get_digital(config::keybinds::wallmech_up) 
        || robot::master.get_digital(config::keybinds::wallmech_down)) {
        set_state = State::OVERRIDE;
    } else if (set_state == State::OVERRIDE && (next_stage || rest)) 
    {
        set_state = State::RESTING;
    }

    switch (set_state) {
        case State::RESTING: {
            if (next_stage && !last_next_stage) {
                set_state = State::PRIMED;
            }
            break;
        }
        case State::PRIMED: {
            if (rest && !last_rest) {
                set_state = State::RESTING;
            } else if (next_stage && !last_next_stage) {
                set_state = State::FIRING;
                special_fire_thing = pros::millis() + 100;
            }
            break;
        }
        case State::FIRING: {
            if (rest && !last_rest) {
                set_state = State::RESTING;
            } else if (next_stage && !last_next_stage) {
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

    last_next_stage = next_stage;
    last_rest = rest;

    if (set_state == State::OVERRIDE) return;

    if (special_fire_thing != -1) {
        if (pros::millis() < special_fire_thing) {
            robot::conveyor.move_voltage(-12000);
        } else {
            special_fire_thing = -1;
            robot::conveyor.move_voltage(0);
        }
    }

    const auto& desired = positions[(int) set_state];
    const auto current = robot::wallmech.get_position();
    
    if (std::abs(desired - current) < 15.0f) {
        robot::wallmech.brake();
    } else {
        float control = pid.get(desired - current);
        control = std::clamp(control, -1.0f, 1.0f);
        robot::wallmech.move_voltage((int) roundf(control * 12000));
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