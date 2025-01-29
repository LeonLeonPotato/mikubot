#pragma once

#include "api.h" // IWYU pragma: keep

#include "essential.h" // IWYU pragma: export
#include "config.h" // IWYU pragma: export
#include "autonomous/controllers.h" // IWYU pragma: export
#include "autonomous/movement.h" // IWYU pragma: export
#include "autonomous/pathing.h" // IWYU pragma: export

// One VEX tile in cm
#define TILE 59.5f
#define pi M_PI

static controllers::PID linear_pid({.kp = 1.5 / 45, .ki = 0.0, .kd = 0.002f});
static controllers::PID linear_boomerang_pid({.kp = 0.015, .ki = 0.1, .kd = 0.01f});

static controllers::PID angular_pid({.kp = 1.0, .ki = 1.0, .kd = 0.1});
static controllers::PID angular_boomerang_pid({.kp = 2.0, .ki = 1.0, .kd = 0.1});

static controllers::PID in_place_pid({.kp = 1.0, .ki = 0.1, .kd = 0.1});

static const movement::PIDGroup swing_group {linear_pid, angular_pid};
static const movement::PIDGroup boomerang_group {linear_boomerang_pid, angular_boomerang_pid};

static const pathing::ProfileParams profile_params {
    .start_v = 10,
    .end_v = 0,
    .max_speed = 100, // true: around 140?
    .accel = 200, // true: 427.376053809
    .decel = 200,
    .track_width = 39,
    .ds = 0.1,
    .resolution = 5000
};

static void print_poses(const std::vector<std::pair<Eigen::Vector2f, float>>& poses) {
    std::cout << "P_p = \\left[";
    for (int i = 0 ; i < poses.size(); i++) {
        auto& pos = poses[i].first;
        std::cout << "\\left(" << pos.x() << ",\\ " << pos.y() << "\\right)";
        if (i != poses.size() - 1) {
            std::cout << ", ";
        }
        pros::delay(20);
    }
    std::cout << "\\right]" << std::endl;
    pros::delay(100);

    std::cout << "P_t = \\left[";
    for (int i = 0 ; i < poses.size(); i++) {
        auto& pos = poses[i].second;
        std::cout << pos;
        if (i != poses.size() - 1) {
            std::cout << ", ";
        }
        pros::delay(20);
    }
    std::cout << "\\right]" << std::endl;
}

#define get_logging_task(vec) pros::Task([&poses] () { \
    while (true) { \
        printf("pos: %f, %f | angle: %f\n", robot::x(), robot::y(), robot::theta); \
        vec.push_back({ \
            {robot::x(), robot::y()}, \
            robot::theta \
        }); \
        pros::delay(20); \
    } \
});