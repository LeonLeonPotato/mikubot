#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

void test_strategy::run(void) {
    robot::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    auto qs = pathing::QuinticSpline();
    qs.points.emplace_back(robot::x, robot::y);
    qs.points.emplace_back(robot::x, robot::y + 100);
    qs.points.emplace_back(robot::x + 50, robot::y + 200);
    qs.points.emplace_back(robot::x + 200, robot::y + 200);

    pathing::BaseParams qsparams = {0, 0, 0, 0};
    movement::PurePursuit(qs, qsparams, 10).follow_path();

    movement::turn_towards(-M_PI/2, 0.1);
    robot::brake();

    auto pathback = pathing::BoomerangPath(
        Eigen::Vector2f(robot::x, robot::y), Eigen::Vector2f(0, 0)
    );

    pathing::BaseParams pbparams = {0, 0, -M_PI, 1 / sqrtf(2)};
    movement::PurePursuit(pathback, pbparams, 10).follow_path();

    robot::brake();
    robot::set_brake_mode(robot::config::default_brake_mode);
}