#include "autonomous/strategy/test.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"

using namespace strategies;

void test_strategy::run(void) {
    std::vector<Pose> poses;
    auto logger_task = get_logging_task(poses);
    // auto res = movement::simple::boomerang(
    //     {50, 50, pi/2}, 
    //     0.5f, 
    //     {.reversed = false, .linear_exit_threshold=2.0, .timeout=10000}, 
    //     boomerang_group);
    robot::chassis.take_drive_mutex();

    // auto res = movement::simple::turn_towards(
    //     -pi,
    //     {.angular_exit_threshold = -1, .timeout = 5000},
    //     in_place_pid
    // );

    // auto res = movement::simple::forward(
    //     100,
    //     {.linear_exit_threshold = -1, .timeout = 5000},
    //     linear_pid
    // );

    auto res = movement::simple::boomerang(
        {50, 50, pi/2}, 
        0.3f, 
        {.linear_exit_threshold=2.0, .angular_exit_threshold=999, .max_linear_speed=0.5f, .timeout=2000}, 
        boomerang_group);

    robot::brake();
    logger_task.remove();

    pros::delay(20);
    printf("Test strategy finished with status: \n%s\n", res.debug_out().c_str());
    robot::chassis.give_drive_mutex();

    pros::delay(100);

    print_poses(poses);
}

