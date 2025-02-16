#pragma once

#include "api.h" // IWYU pragma: keep

#include "essential.h" // IWYU pragma: export
#include "config.h" // IWYU pragma: export
#include "autonomous/controllers.h" // IWYU pragma: export
#include "autonomous/movement.h" // IWYU pragma: export
#include "autonomous/pathing.h" // IWYU pragma: export

// One VEX tile in cm
#define TILE 60.0f
#define pi ((float) M_PI)

static controllers::PID linear_pid({.kp = 0.022, .ki = 0.06, .kd = 0.003, .disable_integral_limit = 20});
static controllers::PID linear_boomerang_pid({.kp = 0.022, .ki = 0.1, .kd = 0.003, .disable_integral_limit = 20});

static controllers::PID angular_pid({.kp = 1.0f, .ki = 0.00, .kd = 0.040625f});
static controllers::PID angular_boomerang_pid({.kp = 1.0f, .ki = 0.00, .kd = 0.040625f});

static controllers::PID in_place_pid({.kp = 1.0f, .ki = 0.00, .kd = 0.040625f});
// static controllers::PID in_place_pid({.kp = 0.0369241673546, .ki = 0.265749949547, .kd = 0.00120207358});

static const movement::PIDGroup swing_group {linear_pid, angular_pid};
static const movement::PIDGroup boomerang_group {linear_boomerang_pid, angular_boomerang_pid};

static const pathing::ProfileParams profile_params {
    .start_v = 10,
    .end_v = 0,
    .max_speed = 100, // true: around 140?
    .accel = 200, // true: 427.376053809
    .decel = 200,
    .track_width = 39,
    .friction_coeff = 0.5,
    .ds = 0.1
};

static void print_poses(const std::vector<Pose>& poses) {
    for (const auto& pose : poses) {
        printf("%.3f, %.3f, %.3f\n", pose.x(), pose.y(), pose.theta());
        pros::delay(50);
    }
}

#define get_logging_task(vec) pros::Task([&poses] () { \
    while (true) { \
        vec.push_back(robot::get_pose()); \
        pros::delay(20); \
    } \
});