#include "bcis.h"
#include "ansicodes.h"
#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "essential.h"
#include "gui/debugscreen.h"
#include "opcontrol/impl/conveyor.h"
#include "opcontrol/impl/wallmech.h"
#include "pros/rtos.hpp"
#include "autonomous/strategy/utils.h"
#include <iostream>

using namespace strategies;

using Vec = Eigen::Vector2f;

constexpr float initial_offset_y = 37.0f;
constexpr float initial_offset_x = 7 + 5.0;

static void alliance_stake(void) {
    // Push middle rings out the way
    movement::simple::forward(
        initial_offset_y + 10, 
        { .reversed=true, .max_linear_speed=0.6f, .timeout=2000 }, 
        linear_pid);

    movement::simple::forward(
        9, 
        { .reversed=false, .max_linear_speed=0.8f, .timeout=1000 }, 
        linear_pid);

    // Turn to back the alliance stake
    movement::simple::turn_towards(
        {-20 * robot::match::side, -initial_offset_y}, 
        { .reversed=true, .timeout=1000 }, 
        in_place_pid);
    
    // Back up into alliance stake

    // movement::simple::swing_to(
    //     {-15 * robot::match::side, -initial_offset_y}, 
    //     {.reversed=true, .max_linear_speed=0.5f, .timeout=2000},
    //     swing_group);

    movement::simple::forward(
        9, 
        { .reversed=true, .max_linear_speed=0.5f, .timeout=1000 }, 
        linear_pid);
    
    robot::brake();

    // Run conveyor and score
    controls::conveyor::exposed_set_color_sort(false);
    controls::conveyor::exposed_desired_volt(12000);
    pros::delay(1000);
    controls::conveyor::exposed_desired_volt(0);
    controls::conveyor::exposed_set_color_sort(true);
}

static void get_the_mobile_goal(void) {
    // Goal pos and approach angle
    Vec goal_pos {50.4 * robot::match::side, 30};
    float angle = pi/2 * robot::match::side;

    // Roughly turn so we dont swing wildly when boomeranging
    movement::simple::turn_towards(
        pi, 
        {.reversed=false, .angular_exit_threshold=rad(5), .timeout=1500}, 
        in_place_pid);

    // Boomerang over there
    movement::simple::boomerang(
        goal_pos, 
        angle, 
        0.3f, 
        {.reversed = true, .linear_exit_threshold=2.5f}, 
        boomerang_group);

    pros::delay(300);
    
    // Clamp down
    robot::clamp.extend();

    pros::delay(200);

    robot::brake();
}

static void get_first_ring(void) {
    // Ring pos
    Vec ring_pos {55 * robot::match::side, 87};

    // Look at the ring
    movement::simple::turn_towards(
        ring_pos, 
        {.angular_exit_threshold = rad(2.5), .timeout = 1000}, 
        in_place_pid);

    // Async so we start moving before we activate conveyor to prevent bad stuff happening
    auto future = movement::simple::forward_async(
        TILE, 
        {.timeout = 3000}, 
        linear_pid);

    pros::delay(200);

    // Start conveyor and intake
    robot::intake.move_voltage(12000);
    controls::conveyor::exposed_desired_volt(12000);

    future.wait(); // Wait to get there

    robot::intake.move_voltage(0);
    controls::conveyor::exposed_desired_volt(0);

    robot::brake();
}

void bcis::run(void) {
    std::vector<std::pair<Eigen::Vector2f, float>> poses;
    pros::Task logging_task([&] () {
        while (true) {
            printf("pos: %f, %f | angle: %f\n", robot::pos.x(), robot::pos.y(), robot::theta);
            poses.push_back({
                {robot::pos.x(), robot::pos.y()},
                robot::theta
            });
            pros::delay(20);
        }
    });

    std::cout << PREFIX << "Running BCIS strategy\n";

    printf("%sMultiplier: %d", CPREFIX, robot::match::side);
    
    controls::wallmech::start_api_task();
    controls::conveyor::start_api_task();

    alliance_stake();
    // get_the_mobile_goal();
    // get_first_ring();

    robot::brake();

    std::cout << PREFIX << "BCIS strategy finished\n";

    print_poses(poses);
    logging_task.remove();
}