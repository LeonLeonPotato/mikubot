#include "subsystems/impl/wallmech.h"
#include "autonomous/controllers/pid.h"
#include "conveyor.h"
#include "essential.h"
#include "config.h"
#include "api.h"
#include "pros/rtos.hpp"

using namespace subsystems;
using State = WallMech::State;

WallMech* WallMech::instance = nullptr;
float WallMech::positions[3] = {0.0f, 50.0f, 170.0f};

void WallMech::api_tick(void) {
    auto& conveyor = Conveyor::get_instance();

    if (set_state == State::OVERRIDE) {
        robot::wallmech.release_mutex();
        return;
    } else if (!robot::wallmech.poll_mutex()) {
        robot::wallmech.acquire_mutex();
    }

    if (special_fire_thing != -1) {
        if (pros::millis() < special_fire_thing) {
            conveyor.set_override_mode(true);
            conveyor.set_override_voltage(-3000);
        } else {
            special_fire_thing = -1;
            conveyor.set_override_mode(false);
            conveyor.set_override_voltage(0);
        }
    }

    const auto& desired = positions[(int) set_state];
    const auto current = robot::wallmech_encoder.get_position() / 100.0f;

    if (set_state == State::FIRING) {
        pid.args.kp = 1.0f;
    } else {
        pid.args.kp = 0.0115f;
    }

    if (std::abs(desired - current) < 1.0f) {
        robot::wallmech.brake();
    } else {
        float control = pid.get(desired - current);
        control = std::clamp(control, -1.0f, 1.0f);
        robot::wallmech.set_desired_voltage(control * 12000);
    }
}

void WallMech::tick(void) {
    if (!poll_mutex()) return;

    bool next_stage = robot::master.get_digital_new_press(config::keybinds::wallmech_next_stage);
    bool rest = robot::master.get_digital_new_press(config::keybinds::wallmech_rest);

    if (robot::master.get_digital(config::keybinds::wallmech_up) 
        || robot::master.get_digital(config::keybinds::wallmech_down)) {
        set_state = State::OVERRIDE;
    } else if (set_state == State::OVERRIDE && (next_stage || rest)) 
    {
        robot::wallmech.release_mutex();
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
            robot::wallmech.acquire_mutex();

            const int speed = (robot::master.get_digital(config::keybinds::wallmech_up) 
                - robot::master.get_digital(config::keybinds::wallmech_down)) 
                * 12000;

            if (speed == 0) robot::wallmech.brake();
            else robot::wallmech.set_desired_voltage(speed);
            break;
        }
    }
}

void WallMech::exposed_go_to_state(State state) {
    if (!poll_mutex()) return;

    set_state = state;

    if (set_state == State::FIRING) {
        special_fire_thing = pros::millis() + 100;
    }
}