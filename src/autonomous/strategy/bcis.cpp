#include "bcis.h"
#include "ansicodes.h"
#include "autonomous/movement/simple/boomerang.h"
#include "essential.h"
#include "opcontrol/impl/wallmech.h"
#include "pros/rtos.hpp"
#include "autonomous/strategy/utils.h"
#include <iostream>

using namespace strategies;

#define pi M_PI
#define reset_all() robot::brake(); swing_group.reset(); linear_pid.reset(); in_place_pid.reset();

constexpr float initial_offset_y = -17.0f;
constexpr float initial_offset_x = 40.0f;

static void alliance_stake(void) {
    // Push middle rings out the way
    movement::simple::forward(initial_offset_x + 5, { .reversed=true, .max_linear_speed=0.5f, .timeout=2000 }, linear_pid);
    linear_pid.reset();

    // Move to the alliance stake
    movement::simple::forward(5, {.exit_threshold=1.0f, .max_linear_speed = 0.5f, .timeout=1000 }, linear_pid);
    linear_pid.reset();

    // Turn to back the alliance stake
    movement::simple::turn_towards(-pi/2, { .exit_threshold=rad(2), .timeout=2000 }, in_place_pid);
    in_place_pid.reset();
    
    movement::simple::forward(10, { .reversed=true, .exit_threshold=0.1f, .max_linear_speed=1.0f, .timeout=750 }, linear_pid);
    reset_all();

    // Run conveyor and score
    robot::conveyor.move_voltage(10000);
    pros::delay(750);
    robot::conveyor.move_voltage(0);

    // We are at 8.5, -40.5
}

static void get_the_mobile_goal(void) {
    Eigen::Vector2f goal_pos {89.5, TILE + initial_offset_y};
    float goal_angle = pi/2;
    // Boomerang so we get there facing backwards
    movement::simple::boomerang(goal_pos, goal_angle, 0.5f, 
        { .reversed=false, .max_linear_speed=0.5f, .timeout=1000 }, swing_group);
    swing_group.reset();

    // Move "into" the goal delay
    pros::delay(100);
    robot::clamp.extend();
    pros::delay(250); // For safety

    robot::brake();
}

static void get_first_ring(void) {
    Eigen::Vector2f ring_pos {89.5, 2*TILE + initial_offset_y};
    // Look at the ring
    movement::simple::turn_towards(ring_pos, {.exit_threshold = rad(2.5), .timeout = 1000}, in_place_pid);
    swing_group.reset();

    // Async so we start moving before we activate conveyor to prevent bad stuff happening
    auto future = movement::simple::forward_async(TILE * 1.5, {.max_linear_speed = 1.0f, .timeout = 1000}, linear_pid);
    pros::delay(200);

    // Start conveyor
    robot::intake.move_voltage(12000);
    robot::conveyor.move_voltage(12000);
    future.get(); // Wait to get there

    // robot::intake.move_voltage(0);
    // robot::conveyor.move_voltage(0);
    reset_all();
}

void bcis::run(void) {
    std::cout << PREFIX << "Running BCIS strategy\n";
    
    controls::wallmech::start_api_task();

    // Back up and score on the alliance stake
    // alliance_stake();
    // get_the_mobile_goal();
    // get_first_ring();
    // Boomerang to get the mobile goal

    movement::simple::boomerang({-50, -50}, -pi/2, 0.5f, {.reversed = true, .exit_threshold=10.0f, .max_linear_speed=1.0f, .timeout=5000}, swing_group);

    reset_all();


    std::cout << PREFIX << "BCIS strategy finished\n";
}