#include "autonomous/strategy/test.h"
#include "autonomous/strategy/utils.h"

using namespace strategies;

static float START_OFFSET = 17.0f;

static void get_goal(void) {
    movement::simple::swing_to({0, -2*TILE + START_OFFSET}, {.reversed=true}, swing_group);
    swing_group.reset(); robot::brake();
}

static void long_path_part(void) {
    pathing::QuinticSpline path; // Empty quintic spline
    path.points.emplace_back(0, 0);
    path.points.emplace_back(0, TILE);
    path.points.emplace_back(-TILE, TILE);
    path.points.emplace_back(-TILE+7, 0);
    path.points.emplace_back(-TILE-5, -TILE+10);
    path.set_relative({0, -2*TILE + START_OFFSET});
    path.points.insert(path.points.begin(), robot::pos);

    path.solve_coeffs({.start_heading=robot::theta, .start_magnitude=0, .end_heading=M_PI, .end_magnitude=10});
    path.profile_path(profile_params);

    movement::ramsete::RamseteParams params { 
        {
            .exit_threshold=10.0,
            .timeout=10000
        }, 
        {
            .beta=0.5,
            .zeta=0.7
        } 
    };

    auto res = movement::ramsete::follow_path(path, params);
    robot::brake();
}

static void go_back(void) {
    auto target = robot::pos + Eigen::Vector2f {0, TILE};
    movement::SimpleMovementParams params {
        .reversed = true, 
        .exit_threshold = 10.0f, 
        .max_linear_speed = 0.8f
    };
    movement::simple::swing_to(target, params, swing_group);
    swing_group.reset(); robot::brake();
}

void test_strategy::run(void) {
    get_goal();
    long_path_part();
    go_back();

    movement::simple::turn_towards(robot::theta - rad(20), 
    {
        .exit_threshold=rad(5),
        .timeout=5000
    }, in_place_pid);
    in_place_pid.reset(); robot::brake();

    // movement::simple::swing_to(robot::pos + Eigen::Vector2f {-10, -TILE}, {.reversed=false, .max_linear_speed=0.8f}, swing_group);
    // swing_group.reset(); robot::brake();

    robot::brake();
}

