#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple.h"
#include "autonomous/pathing.h"
#include "autonomous/pathing/base_path.h"
#include "essential.h"
#include "config.h" // IWYU pragma: keep

#include "api.h" // IWYU pragma: keep

#define TILE 59.5f
#define THETA_SOLVE pathing::BaseParams { .start_heading = robot::theta, .start_magnitude = 20, .end_heading = 0, .end_magnitude = 0 }
#define rad(x) (x * M_PI / 180.0f)

using namespace strategies;

static const controllers::PIDArgs linear_args {
    .kp = 1.0f / 45.0f,
    .ki = 0,
    .kd = -0.001f
};

static const controllers::PIDArgs angular_args {
    .kp = 1.0,
    .ki = 0,
    .kd = -0.0
};

static const controllers::PIDArgs in_place_args {
    .kp = 1.0,
    .ki = 0,
    .kd = -0.0
};

static controllers::PID linear_pid(linear_args);
static controllers::PID angular_pid(angular_args);
static controllers::PID in_place_pid(in_place_args);

static const movement::PIDGroup path_group {
    .angular = angular_pid,
    .linear = linear_pid
};

static const movement::PIDGroup swing_group {
    .angular = angular_pid,
    .linear = linear_pid
};

static const pathing::ProfileParams profile_params {
    .start_v = 10,
    .end_v = 0,
    .max_speed = 120,
    .accel = 250,
    .decel = 200,
    .track_width = 39,
    .ds = 0.1,
    .resolution = 5000
};

void test_strategy::run(void) {
    pathing::QuinticSpline path; // Empty quintic spline
    path.points.emplace_back(0, 0);
    path.points.emplace_back(0, TILE);
    path.points.emplace_back(-TILE, TILE);
    path.points.emplace_back(-TILE+7, 0);
    path.points.emplace_back(-TILE-5, -TILE+10);
    path.set_relative({0, -2*TILE+17});
    movement::simple::swing_to({0, -2*TILE+17}, {.reversed=true}, swing_group);
    swing_group.reset(); robot::brake();

    path.solve_coeffs({.start_heading=robot::theta, .start_magnitude=0, .end_heading=M_PI, .end_magnitude=10});
    path.profile_path(profile_params);

    movement::ramsete::RamseteParams params {{
        .exit_threshold=10.0,
        .timeout=10000
    }, {
        .beta=0.5,
        .zeta=0.7
    }};

    auto res = movement::ramsete::follow_path(path, params);
    robot::brake();

    movement::simple::swing_to(robot::pos + Eigen::Vector2f {0, TILE}, {.reversed=true, .exit_threshold=10.0f, .max_linear_speed=0.8f}, swing_group);
    swing_group.reset(); robot::brake();

    movement::simple::turn_towards(robot::theta + rad(20), 
    {
        .exit_threshold=rad(5),
        .timeout=5000
    }, in_place_pid);
    in_place_pid.reset(); robot::brake();

    // movement::simple::swing_to(robot::pos + Eigen::Vector2f {-10, -TILE}, {.reversed=false, .max_linear_speed=0.8f}, swing_group);
    // swing_group.reset(); robot::brake();

    robot::brake();
}

