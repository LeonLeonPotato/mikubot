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

static void start_conveyor(void) {
    robot::intake.move_voltage(12000);
    controls::conveyor::exposed_desired_volt(10000);
}

static void stop_conveyor(void) {
    robot::intake.move_voltage(0);
    controls::conveyor::exposed_desired_volt(0);
}

static void get_goal(void) {
    movement::simple::forward(
        80, 
        {.reversed=true, .max_linear_speed=0.6f, .timeout=1000}, 
        linear_pid);

    pros::delay(200);    
    robot::clamp.extend();
    pros::delay(200);
    
    movement::simple::forward(
        15, 
        {.reversed=false, .timeout=500}, 
        linear_pid);

    start_conveyor();
    pros::delay(750);
    stop_conveyor();
}

static void get_big_ring_stack(void) {
    Vec ring_close {
        0 * robot::match::side,
        0
    };
    Vec ring_far {
        0 * robot::match::side,
        0
    };
}

static void get_single_ring(void) {
    Vec ring {
        -60 * robot::match::side,
        -83
    };

    movement::simple::turn_towards(
        ring, 
        {
            .angular_exit_threshold=rad(2.5f), 
            .timeout=1000
        },
        in_place_pid);

    auto fut = movement::simple::forward_async(
        TILE * 1.3, 
        {
            .linear_exit_threshold=1.0f, 
            .timeout=2000
        },
        linear_pid);
    
    pros::delay(100);
    start_conveyor();
    fut.wait();
    pros::delay(1000);
    stop_conveyor();
}

static void goal_rush(void) {
    Vec goal {
        0 * robot::match::side,
        0
    };

    movement::simple::turn_towards(
        goal, 
        {
            .angular_exit_threshold=rad(2.5f), 
            .timeout=1000
        },
        in_place_pid);

    movement::simple::forward(
        TILE, 
        {
            .linear_exit_threshold=1.0f, 
            .timeout=2000
        },
        linear_pid);
    
    robot::doinker.extend();
    pros::delay(100);

    movement::simple::forward(
        1.5 * TILE, 
        {
            .reversed=true,
            .linear_exit_threshold=1.0f, 
            .timeout=2000
        },
        linear_pid);

    robot::doinker.retract();

    movement::simple::forward(
        0.5 * TILE, 
        {
            .linear_exit_threshold=1.0f, 
            .timeout=2000
        },
        linear_pid);
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

    get_goal();
    if ((robot::match::side == -1 && robot::match::team == 'B') || (robot::match::side == 1 && robot::match::team == 'R')) {
        get_big_ring_stack();
    }
    get_single_ring();
    if ((robot::match::side == 1 && robot::match::team == 'B') || (robot::match::side == -1 && robot::match::team == 'R')) {
        goal_rush();
    }

    robot::brake();

    std::cout << PREFIX << "BCIS strategy finished\n";

    print_poses(poses);
    logging_task.remove();
}