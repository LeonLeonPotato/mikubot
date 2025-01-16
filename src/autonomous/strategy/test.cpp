#include "autonomous/strategy/test.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"

using namespace strategies;

void test_strategy::run(void) {
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

    movement::simple::swing_to(
        {0, -25}, 
        {.reversed = true, .linear_exit_threshold=2.0, .timeout=10000}, 
        swing_group);

    robot::brake();
    pros::delay(1000);

    movement::simple::swing_to(
        {0, 0}, 
        {.linear_exit_threshold=2.0, .timeout=10000}, 
        swing_group);

    movement::simple::boomerang(
        {-50, -50}, 
        -pi/2, 
        0.5f, 
        {.reversed = true, .linear_exit_threshold=2.0, .timeout=10000}, 
        boomerang_group);

    robot::brake();

    print_poses(poses);
    logging_task.remove();
}

