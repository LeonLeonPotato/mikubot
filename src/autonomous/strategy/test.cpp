#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

void test_strategy::run(void) {
    auto qs = pathing::QuinticSpline();
    qs.points.emplace_back(robot::x, robot::y);
    qs.points.emplace_back(robot::x, robot::y + 100);
    qs.points.emplace_back(robot::x + 50, robot::y + 200);
    qs.points.emplace_back(robot::x + 200, robot::y + 200);

    pathing::BaseParams qsparams = {0, 0, 0, 0};

    movement::pure_pursuit::follow_path(qs, qsparams, 30);
    movement::turn_towards(-M_PI/2, 0.1);
    robot::brake();

    auto pathback = pathing::BoomerangPath(
        Eigen::Vector2f(robot::x, robot::y), Eigen::Vector2f(0, 0)
    );

    movement::pure_pursuit::follow_path(pathback, 30, nullptr, -M_PI, 1 / sqrtf(2), 5);
}