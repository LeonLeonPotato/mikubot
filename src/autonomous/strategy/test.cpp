#include "autonomous/strategy/test.h"
#include "autonomous/strategy/utils.h"

using namespace strategies;

static float START_OFFSET = 17.0f;

static void get_goal(void) {
    movement::simple::swing_to({0, -2*TILE + START_OFFSET}, {.reversed=true}, swing_group);
    swing_group.reset(); robot::brake();
}

static void long_path_part(void) {
    pathing::QuinticSpline path;
    path.points.emplace_back(-TILE * 2/3.0, TILE);
    path.points.emplace_back(-TILE, 0);
    path.points.emplace_back(-TILE-5, -TILE+10);
    path.set_relative({0, -2*TILE + START_OFFSET});
    path.points.insert(path.points.begin(), robot::pos);

    path.solve_coeffs({.start_heading=0, .start_magnitude=100, .end_heading=M_PI, .end_magnitude=10});
    path.profile_path(profile_params);
    std::cout << path.debug_out() << std::endl;

    movement::ramsete::RamseteParams params { 
        {
            .exit_threshold=5.0,
            .timeout=10000
        }, 
        {
            .beta=2.0,
            .zeta=0.7
        } 
    };

    auto res = movement::ramsete::follow_path(path, params);
    robot::brake();
}

static void go_back(void) {
    Eigen::Vector2f target = robot::pos + Eigen::Vector2f {0, TILE};
    movement::SimpleMovementParams params {
        .reversed = true, 
        .exit_threshold = 10.0f, 
        .max_linear_speed = 0.8f
    };
    auto res = movement::simple::swing_to(target, params, swing_group);
    swing_group.reset(); robot::brake();
}

static void get_second_shit(void) {
    Eigen::Vector2f second = {-TILE + 5, -3*TILE + START_OFFSET + 10};
    movement::simple::turn_towards(
        second, 
        {
            .exit_threshold=rad(5),
            .timeout=5000
        }, 
        in_place_pid
    );
    in_place_pid.reset(); robot::brake();

    movement::SimpleMovementParams params {
        .reversed = false, 
        .exit_threshold = 5.0f, 
        .max_linear_speed = 0.8f
    };
    auto res = movement::simple::swing_to(second, params, swing_group);
    swing_group.reset(); robot::brake();
}

static void test_ramsete(void) {
    pathing::QuinticSpline path;
    path.points.emplace_back(0, 0);
    path.points.emplace_back(0, TILE);
    path.points.emplace_back(TILE, TILE);
    path.points.emplace_back(TILE, 0);
    path.set_relative(robot::pos);

    path.solve_coeffs({.start_heading=robot::theta, .start_magnitude=0, .end_heading=0, .end_magnitude=0});
    path.profile_path(profile_params);

    movement::ramsete::RamseteParams params { 
        {
            .exit_threshold=3.0,
            .timeout=10000
        }, 
        {
            .beta=2.0,
            .zeta=0.7
        } 
    };

    auto res = movement::ramsete::follow_path(path, params);
    robot::brake();
}

void test_strategy::run(void) {
    //test_ramsete();
    get_goal();
    long_path_part();
    go_back();
    get_second_shit();
    go_back();

    robot::brake();
}

