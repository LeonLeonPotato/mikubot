#pragma once

#include "api.h" // IWYU pragma: keep

#include "essential.h" // IWYU pragma: export
#include "config.h" // IWYU pragma: export
#include "autonomous/controllers.h" // IWYU pragma: export
#include "autonomous/movement.h" // IWYU pragma: export
#include "autonomous/pathing.h" // IWYU pragma: export

// One VEX tile in cm
#define TILE 59.5f

static const controllers::PIDArgs linear_args {
    .kp = 1.0 / 45.0,
    .ki = 0,
    .kd = 0.002f
};

static const controllers::PIDArgs angular_args {
    .kp = 0.5,
    .ki = 0,
    .kd = 0.08
};

static const controllers::PIDArgs in_place_args {
    .kp = 0.9,
    .ki = 0.0,
    .kd = 0.1
};

static controllers::PID linear_pid(linear_args);
static controllers::PID angular_pid(angular_args);
static controllers::PID in_place_pid(in_place_args);

static const movement::PIDGroup path_group {linear_pid, angular_pid};
static const movement::PIDGroup swing_group {linear_pid, angular_pid};

static const pathing::ProfileParams profile_params {
    .start_v = 10,
    .end_v = 0,
    .max_speed = 100, // true: around 150?
    .accel = 200, // true: 427.376053809
    .decel = 200,
    .track_width = 39,
    .ds = 0.1,
    .resolution = 5000
};