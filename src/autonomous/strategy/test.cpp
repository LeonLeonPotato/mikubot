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
    pros::Task logging_task = get_logging_task(poses);

    auto res = movement::simple::boomerang(
        {50, 50}, 
        pi/2, 
        0.5f, 
        {.reversed = false, .linear_exit_threshold=2.0, .timeout=10000}, 
        boomerang_group);

    robot::brake();

    logging_task.remove();
    print_poses(poses);

    pros::delay(20);
    printf("Test strategy finished with status: \n%s\n", res.debug_out().c_str());
}

