#include "subsystems/impl/wallmech.h"
#include "autonomous/controllers/pid.h"
#include "conveyor.h"
#include "essential.h"
#include "config.h"
#include "api.h"
#include "pros/rtos.hpp"

using namespace subsystems;
using State = WallMech::State;

float WallMech::positions[3] = {0.0f, 45.9f, 175.9f};

void WallMech::api_tick(void) {
    if (set_state == State::OVERRIDE) {
        robot::wallmech.release_mutex();
        return;
    } else if (!robot::wallmech.poll_mutex()) {
        robot::wallmech.acquire_mutex();
    }

    if (special_fire_thing != -1) {
        if (!Conveyor::get_instance().poll_mutex()) Conveyor::get_instance().take_mutex();

        if (pros::millis() < special_fire_thing) {
            Conveyor::get_instance().set_desired_voltage(-6000);
        } else {
            special_fire_thing = -1;
            Conveyor::get_instance().set_desired_voltage(0);
        }
    } else if (Conveyor::get_instance().poll_mutex()) {
        Conveyor::get_instance().give_mutex();
    }

    const auto& desired = positions[(int) set_state];
    const auto current = robot::wallmech_encoder.get_position() / 100.0f;

    if (std::abs(desired - current) < 2.5f) {
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