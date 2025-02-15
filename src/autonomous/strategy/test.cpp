#include "autonomous/strategy/test.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"
#include "subsystems.h"
#include "subsystems/impl/clamp.h"
#include "subsystems/impl/conveyor.h"

using namespace strategies;

void test_strategy::run(void) {
    robot::chassis.take_drive_mutex();

    std::vector<Pose> poses;
    auto logger_task = get_logging_task(poses);
    // auto res = movement::simple::boomerang(
    //     {50, 50, pi/2}, 
    //     0.5f, 
    //     {.reversed = false, .linear_exit_threshold=2.0, .timeout=10000}, 
    //     boomerang_group);

    // auto res = movement::simple::turn_towards(
    //     -pi/2,
    //     {.angular_exit_threshold = -1, .timeout = 5000},
    //     in_place_pid
    // );

    auto res1 = movement::simple::swing_to(
        {0, -20},
        {.reversed=true, .linear_exit_threshold = 1, .timeout = 5000},
        swing_group
    );

    auto res2 = movement::simple::turn_towards(
        -pi/2,
        {.angular_exit_threshold = -1, .timeout = 800},
        in_place_pid
    );

    auto res3 = movement::simple::forward(
        10,
        {.reversed=true, .linear_exit_threshold = 0.3, .timeout = 500},
        linear_pid
    );

    robot::brake();

    auto& conveyor = subsystems::Conveyor::get_instance();
    conveyor.take_mutex();
    conveyor.set_desired_voltage(12000);
    pros::delay(700);
    conveyor.set_desired_voltage(0);

    auto res4 = movement::simple::turn_towards(
        pi,
        {.angular_exit_threshold = -1, .timeout = 800},
        in_place_pid
    );

    auto res5 = movement::simple::boomerang(
        {-75, 30, -pi/2}, 
        0.6f, 
        {.reversed=true, .linear_exit_threshold=4.0, .angular_exit_threshold=999, .max_linear_speed=1.0f, .timeout=3000}, 
        boomerang_group);

    robot::clamp.extend();
    pros::delay(100);

    auto res6 = movement::simple::turn_towards(
        -pi/3,
        {.angular_exit_threshold = -1, .timeout = 800},
        in_place_pid
    );

    conveyor.set_desired_voltage(12000);
    auto res7 = movement::simple::boomerang(
        {-130, 96, 0},
        0.3f,
        {.linear_exit_threshold=5.0, .angular_exit_threshold=999, .max_linear_speed=1.0f, .timeout=3000},
        swing_group
    );

    robot::brake();
    pros::delay(1000);

    auto res8 = movement::simple::turn_towards(
        0,
        {.angular_exit_threshold = -1, .timeout = 800},
        in_place_pid
    );

    auto res9 = movement::simple::forward(
        20,
        {.reversed=false, .linear_exit_threshold = 0.3, .timeout = 500},
        linear_pid
    );

    robot::brake();


    logger_task.remove();

    pros::delay(20);
    printf("Test strategy finished with status: \n%s\n", res1.debug_out().c_str());
    robot::chassis.give_drive_mutex();

    pros::delay(1000);

    // print_poses(poses);
}

