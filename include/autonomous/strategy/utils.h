#pragma once

#include "api.h" // IWYU pragma: keep

#include "essential.h" // IWYU pragma: export
#include "config.h" // IWYU pragma: export
#include "autonomous/controllers.h" // IWYU pragma: export
#include "autonomous/movement.h" // IWYU pragma: export
#include "autonomous/pathing.h" // IWYU pragma: export

#define TILE 59.5f
#define THETA_SOLVE pathing::BaseParams { .start_heading = robot::theta, .start_magnitude = 20, .end_heading = 0, .end_magnitude = 0 }

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

static const movement::PIDGroup path_group {angular_pid, linear_pid};
static const movement::PIDGroup swing_group {angular_pid, linear_pid};

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