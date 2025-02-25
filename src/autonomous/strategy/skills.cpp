#include "autonomous/strategy/skills.h"
#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"
#include "pose.h"
#include "subsystems/impl/conveyor.h"

using namespace strategies;

using Vec = Eigen::Vector2f;

#define GOAL(side) (Vec {60.0 * side, 44.5})
#define R1(side) (Vec {(120.0 - 10) * side, 100.5})
#define R3(side) (Vec {(120.0 - 10) * side, 16 + TILE/2})
#define R4(side) (Vec {140.4 * side, 43})
#define R5(side) (Vec {10, TILE} + GOAL(side))

static void corner(int side) {
    const auto goal = GOAL(side);
    const auto r1 = R1(side);
    const auto r3 = R3(side);
    const auto r4 = R4(side);

    // alliance stake
    subsystems::Conveyor::get_instance().set_desired_voltage(12000);
    pros::delay(700);
    subsystems::Conveyor::get_instance().set_desired_voltage(0);

    // get goal
    movement::simple::boomerang(
        Pose(goal, rad(45 * side)), 
        0.5f, 
        {.reversed = true, .linear_exit_threshold = 3.0, .angular_exit_threshold = rad(30), .max_linear_speed = 0.5f, .timeout = 1500},
        swing_group);

    // clamp and unstuck
    subsystems::Conveyor::get_instance().set_desired_voltage(-3000);
    robot::clamp.extend();
    pros::delay(100);
    subsystems::Conveyor::get_instance().set_desired_voltage(12000);

    movement::simple::turn_towards(
        R5(side),
        {.angular_exit_threshold = rad(-1), .timeout = 1500},
        in_place_pid);

    movement::simple::forward(
            TILE,
            {.linear_exit_threshold = 0.5f, .max_linear_speed = 1.0f, .timeout = 1500},
            linear_pid
        );

    movement::simple::forward(
            TILE,
            {.reversed = true, .linear_exit_threshold = 0.5f, .max_linear_speed = 0.75f, .timeout = 1500},
            linear_pid
        );

    // get ring 1
    movement::simple::boomerang(
        Pose(r1 + Vec {0, 40}, rad(60 * side)), 
        0.5f, 
        {.linear_exit_threshold = 5.0, .angular_exit_threshold = rad(9999), .timeout = 1500},
        swing_group);

    // get rings 2 and 3
    movement::simple::turn_towards(
        rad(180),
        {.angular_exit_threshold = rad(-1), .timeout = 1500},
        in_place_pid);

    movement::simple::forward(
        robot::distance(r3),
        {.linear_exit_threshold = 0.5f, .max_linear_speed = 0.3f, .timeout = 5000},
        linear_pid
    );

    robot::brake();
    pros::delay(1000);

    movement::simple::forward(
        TILE * 0.75,
        {.linear_exit_threshold = 0.5f, .max_linear_speed = 0.5f, .timeout = 1500},
        linear_pid
    );

    pros::delay(500);

    // get ring 4
    movement::simple::turn_towards(
        rad(45 * side),
        {.angular_exit_threshold = rad(-1), .timeout = 1500},
        in_place_pid);

    movement::simple::forward(
        robot::distance(r4),
        {.linear_exit_threshold = 0.5f, .max_linear_speed = 1.0f, .timeout = 700},
        linear_pid
    );

    // let go of goal
    movement::simple::turn_towards(
        rad(-30 * side),
        {.angular_exit_threshold = rad(-1), .timeout = 700},
        in_place_pid);

    movement::simple::forward(
        30,
        {.reversed = true, .linear_exit_threshold = 0.5f, .max_linear_speed = 0.7f, .timeout = 700},
        linear_pid
    );

    robot::clamp.retract();
    pros::delay(100);

    // unstuck
    movement::simple::forward(
        30,
        {.linear_exit_threshold = 0.5f, .max_linear_speed = 0.5f, .timeout = 300},
        linear_pid
    );

    robot::brake();

    subsystems::Conveyor::get_instance().set_desired_voltage(-12000);
    pros::delay(700);
    subsystems::Conveyor::get_instance().set_desired_voltage(0);
}

void farside(void) {
    movement::simple::swing_to(
        {105, 155},
        {.linear_exit_threshold = 5.0f, .angular_exit_threshold = 999},
        swing_group
    );
}

void skills::run(void) {
    robot::chassis.take_drive_mutex();
    subsystems::Conveyor::get_instance().take_mutex();

    corner(-1);

    // reset to middle of field
    movement::simple::swing_to(
        {0, TILE / 2.0},
        {.linear_exit_threshold = 10.0f, .angular_exit_threshold=999, .max_linear_speed = 1.0f, .timeout = 3000},
        swing_group
    );
    robot::brake();
    pros::delay(500);

    corner(1);
    farside();

    subsystems::Conveyor::get_instance().give_mutex();
    robot::chassis.give_drive_mutex();
}