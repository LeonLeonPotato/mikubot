#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

pathing::QuinticSpline sp;

void test_strategy::run(void) {
    sp = pathing::QuinticSpline();
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(robot::x, robot::y + 100);
    sp.points.emplace_back(robot::x + 50, robot::y + 200);
    sp.points.emplace_back(robot::x + 200, robot::y + 200);

    movement::pure_pursuit::follow_path(sp, 30);
    movement::turn_towards(-M_PI/2, 0.1);
    robot::brake();

    auto pathback = pathing::BoomerangPath(
        Eigen::Vector2f(robot::x, robot::y), Eigen::Vector2f(0, 0)
    );

    movement::pure_pursuit::follow_path(pathback, 30, nullptr, -M_PI, 1 / sqrtf(2), 5);
}