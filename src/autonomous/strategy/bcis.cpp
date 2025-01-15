#include "bcis.h"
#include "ansicodes.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "essential.h"
#include "gui/debugscreen.h"
#include "opcontrol/impl/wallmech.h"
#include "pros/rtos.hpp"
#include "autonomous/strategy/utils.h"
#include <iostream>

using namespace strategies;

#define pi M_PI
#define reset_all() robot::brake(); swing_group.reset(); linear_pid.reset(); in_place_pid.reset();

constexpr float initial_offset_y = 32.0f;
constexpr float initial_offset_x = 6.60 + 5.0;

static void alliance_stake(void) {
    // Push middle rings out the way
    movement::simple::forward(initial_offset_y, { .reversed=true, .max_linear_speed=0.5f, .timeout=2000 }, linear_pid);

    // Turn to back the alliance stake
    movement::simple::turn_towards({20, -initial_offset_y}, { .reversed=true, .timeout=1000 }, in_place_pid);
    
    movement::simple::forward(999, { .reversed=true, .max_linear_speed=0.5f, .timeout=600 }, linear_pid);
    
    movement::simple::forward(7, { .reversed=false, .linear_exit_threshold=0.1f, .max_linear_speed=1.0f, .timeout=1000 }, linear_pid);
    robot::brake();

    // Run conveyor and score
    robot::conveyor.move_voltage(12000);
    pros::delay(750);
    robot::conveyor.move_voltage(0);

    // We are at 8.5, -40.5
}

static void get_the_mobile_goal(void) {
    movement::simple::turn_towards(pi, {.reversed=false, .angular_exit_threshold=rad(5), .timeout=1500}, in_place_pid);
    movement::simple::forward(50, {.reversed = true}, linear_pid);
    robot::brake();

    Eigen::Vector2f goal_pos {-50.4, 30};
    movement::simple::turn_towards(goal_pos, {.reversed=true, .angular_exit_threshold=rad(0.5), .timeout=1500}, in_place_pid);

    auto fut = movement::simple::forward(100, {.reversed=true, .max_linear_speed=0.6f, .timeout=2000}, linear_pid);

    robot::clamp.extend();

    robot::brake();
}

static void get_first_ring(void) {
    Eigen::Vector2f ring_pos {-55, 87};
    // Look at the ring
    movement::simple::turn_towards(ring_pos, {.angular_exit_threshold = rad(2.5), .timeout = 1000}, in_place_pid);
    swing_group.reset();

    // Async so we start moving before we activate conveyor to prevent bad stuff happening
    auto future = movement::simple::forward_async(TILE * 1.5, {.max_linear_speed = 1.0f, .timeout = 3000}, linear_pid);
    pros::delay(200);

    // Start conveyor
    robot::intake.move_voltage(12000);
    robot::conveyor.move_voltage(12000);
    future.get(); // Wait to get there

    // robot::intake.move_voltage(0);
    // robot::conveyor.move_voltage(0);
    robot::brake();
}

void bcis::run(void) {
    std::cout << PREFIX << "Running BCIS strategy\n";
    
    controls::wallmech::start_api_task();

    // Back up and score on the alliance stake
    alliance_stake();
    get_the_mobile_goal();
    get_first_ring();
    // Boomerang to get the mobile goal

    // movement::simple::boomerang({-50, -50}, -pi/2, 0.25f, {.reversed = true, .linear_exit_threshold=1.0f, .timeout=20000}, swing_group);
    // debugscreen::debug_message += "Boomerang done\n";

    reset_all();


    std::cout << PREFIX << "BCIS strategy finished\n";
}