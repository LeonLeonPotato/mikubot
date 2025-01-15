#pragma once

#include "api.h" // IWYU pragma: keep

#include "essential.h" // IWYU pragma: export
#include "config.h" // IWYU pragma: export
#include "autonomous/controllers.h" // IWYU pragma: export
#include "autonomous/movement.h" // IWYU pragma: export
#include "autonomous/pathing.h" // IWYU pragma: export

// One VEX tile in cm
#define TILE 59.5f

static controllers::PID linear_pid({.kp = 1.0 / 45.0, .ki = 1.0 / 90, .kd = 0.002f});
static controllers::PID angular_pid({.kp = 2.0, .ki = 0, .kd = 0.08});
static controllers::PID in_place_pid({.kp = 0.9, .ki = 0.0, .kd = 0.1});

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