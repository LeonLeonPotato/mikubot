#include "autonomous/strategy/peak_strat.h"
#include "ansicodes.h"
#include "api.h"
#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"
#include "opcontrol/impl/conveyor.h"
#include "opcontrol/impl/wallmech.h"
#include "pros/rtos.hpp"

using namespace strategies;

static void get_goal(void) {
    auto fut = movement::simple::forward_async(
        80, 
        {.reversed=true, .max_linear_speed=0.6f, .timeout=1000}, 
        linear_pid);

    fut.wait();
    pros::delay(200);
    
    robot::clamp.extend();
    
    fut.wait();
    pros::delay(200);
    
    fut = movement::simple::forward_async(
        15, 
        {.reversed=false, .max_linear_speed=0.75f, .timeout=500}, 
        linear_pid);

    fut.wait();

    robot::intake.move_voltage(12000);
    controls::conveyor::exposed_desired_volt(10000);

    pros::delay(750);

    robot::intake.move_voltage(0);
    controls::conveyor::exposed_desired_volt(0);
}

static void get_ring(void) {
    int xto = 60 * robot::match::side;
    int yto = -83;

    movement::simple::turn_towards(
        {xto, yto}, 
        {
            .angular_exit_threshold=rad(2.5f), 
            .timeout=1000
        },
        in_place_pid);

    auto fut = movement::simple::forward_async(
        TILE * 1.3, 
        {
            .linear_exit_threshold=0.7f, 
            .timeout=2000
        },
        linear_pid);

    pros::delay(100);
    robot::intake.move_voltage(12000);
    controls::conveyor::exposed_desired_volt(10000);

    fut.wait();
    pros::delay(100);
    robot::brake();
    pros::delay(500);

    fut = movement::simple::forward_async(
        10, 
        {.reversed=true, .max_linear_speed=0.75f, .timeout=500}, 
        linear_pid);

    pros::delay(500);
    fut.wait();

    robot::intake.move_voltage(0);
    controls::conveyor::exposed_desired_volt(0);

    robot::brake();
}

static void corner(void) {
    Eigen::Vector2f target = {97 * robot::match::side, -28};
    movement::simple::turn_towards(
        target,
        {
            .angular_exit_threshold=rad(2.5f),
            .timeout=1000
        },
        in_place_pid);

    movement::simple::forward(
        robot::distance(target), 
        {.linear_exit_threshold=1.0f, .timeout=2000}, 
        linear_pid);

    movement::simple::turn_towards(
        0,
        {
            .angular_exit_threshold=rad(1.0f),
            .timeout=1000
        },
        in_place_pid);
    
    robot::brake();

    pros::delay(100);
    robot::doinker.extend();
    pros::delay(300);

    movement::simple::turn_towards(
        -pi/2 * robot::match::side,
        {
            .angular_exit_threshold=rad(2.5f),
            .timeout=750
        },
        in_place_pid);

    robot::brake();
    robot::doinker.retract();

    movement::simple::turn_towards(
        -pi/4 * robot::match::side,
        {
            .angular_exit_threshold=rad(2.5f),
            .timeout=500
        },
        in_place_pid);

    robot::intake.move_voltage(12000);
    controls::conveyor::exposed_desired_volt(10000);

    movement::simple::forward(
        robot::distance({0, 0}), 
        {.max_linear_speed=0.7f, .timeout = 1000}, 
        linear_pid);

    robot::brake();

    movement::simple::forward(
        TILE, 
        {.reversed = true, .max_linear_speed=0.7f}, 
        linear_pid);

    robot::brake();

    robot::intake.move_voltage(0);
    controls::conveyor::exposed_desired_volt(0);
}

static void left_rings(void) {
    movement::simple::turn_towards(
        -pi, 
    {.timeout=1000}, 
    in_place_pid);

    robot::brake();

    robot::doinker.extend();
    pros::delay(100);

    movement::simple::forward(
        TILE/2, 
        {.reversed=true, .timeout=1000}, 
        linear_pid);

    if ((robot::match::side == 1 && robot::match::team == 'B') 
        || robot::match::side == -1 && robot::match::team == 'R') 
    {
        movement::simple::forward(
            TILE, 
            {.reversed=true, .timeout=1000}, 
            linear_pid);
        return;
    }

    robot::intake.move_voltage(12000);
    controls::conveyor::exposed_desired_volt(10000);

    movement::simple::swing_to(
        robot::pos + Eigen::Vector2f {-30 * robot::match::side, 10}, 
        {.reversed=true, .timeout=1000}, 
        swing_group);

    movement::simple::turn_towards(
        pi/2 - pi/3,
        {.timeout=1000}, 
        in_place_pid);

    robot::brake();
    robot::doinker.retract();
    pros::delay(100);

    movement::simple::turn_towards(pi/2, {.timeout=500}, in_place_pid);

    movement::simple::forward(TILE, {}, linear_pid);
}


void peak_strat::run(void) {
    std::cout << PREFIX << "Running peak strat\n";

    controls::wallmech::start_api_task();
    controls::conveyor::start_api_task();

    get_goal();
    get_ring();
    left_rings();

    robot::brake();

    std::cout << PREFIX << "Finished peak strat\n";
}