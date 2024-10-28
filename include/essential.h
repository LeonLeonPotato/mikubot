#pragma once

#include "api.h"

#include "Eigen/Dense"

namespace robot {
constexpr float TRACKING_WHEEL_RADIUS = 4.1275f;
constexpr float BACK_TRACKING_WHEEL_OFFSET = 7.075f;
constexpr float SIDE_TRACKING_WHEEL_OFFSET = 10.5f;

inline namespace state {
    extern bool braking;
    extern double x, velocity_x, acceleration_x;
    extern double y, velocity_y, acceleration_y;
    extern double theta, angular_velocity, angular_acceleration;

    inline Eigen::Vector2f pos(void) {
        return Eigen::Vector2f(x, y);
    }

    inline float speed(void) {
        return sqrtf(velocity_x * velocity_x + velocity_y * velocity_y);
    }

    inline float accel(void) {
        return sqrtf(acceleration_x * acceleration_x + acceleration_y * acceleration_y);
    }

    inline float angular_diff(float desired) {
        return fmod(desired - fmod(theta, M_TWOPI) + M_PI, M_TWOPI) - M_PI;
    }

    inline float angular_diff(float desired_x, float desired_y) {
        return angular_diff(atan2(desired_x - x, desired_y - y));
    }

    inline float angular_diff(const Eigen::Vector2f& point) {
        return angular_diff(atan2(point(0) - x, point(1) - y));
    }

    inline float distance(float desired_x, float desired_y) {
        return sqrtf((desired_x - x) * (desired_x - x) + (desired_y - y) * (desired_y - y));
    }

    inline float distance(const Eigen::Vector2f& point) {
        return sqrtf((point(0) - x) * (point(0) - x) + (point(1) - y) * (point(1) - y));
    }
} // namespace state

namespace signatures {
    constexpr int blue_ring_id = 0;
    constexpr int red_ring_id = 1;
    constexpr int goal_id = 2;

    extern pros::vision_signature_s_t blue_ring;
    extern pros::vision_signature_s_t red_ring;
    extern pros::vision_signature_s_t goal;
} // namespace signatures

namespace config {
    constexpr bool velo_based_driving = true;
    extern const pros::motor_brake_mode_e_t default_brake_mode;
} // namespace config

extern pros::Controller master;
extern pros::IMU inertial;
extern pros::Rotation side_encoder;
extern pros::Rotation back_encoder;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

extern pros::MotorGroup intake;
extern pros::Motor conveyor;

extern pros::adi::Pneumatics excluder;
extern pros::Optical classifier;

extern pros::Vision vision;

inline int max_speed(void) {
    switch (left_motors.get_gearing()) {
        case pros::MotorGears::blue:
            return 600;
        case pros::MotorGears::green:
            return 200;
        case pros::MotorGears::red:
            return 100;
        default:
            return 200;
    }
}

inline void volt(int left, int right) {
    left = fminf(fmaxf(left, -127), 127);
    right = fminf(fmaxf(right, -127), 127);
    braking = false;
    left_motors.move(left);
    right_motors.move(right);
}

inline void velo(int left, int right) {
    int max = max_speed();
    int lv = (int) (std::clamp(left, -127, 127) / 127.0f * max);
    int rv = (int) (std::clamp(right, -127, 127) / 127.0f * max);
    braking = false;
    left_motors.move_velocity(lv);
    right_motors.move_velocity(rv);
}

inline void velo(float left, float right) {
    int max = max_speed();
    int lv = (int) (std::clamp(left, -1.0f, 1.0f) * max);
    int rv = (int) (std::clamp(right, -1.0f, 1.0f) * max);
    braking = false;
    left_motors.move_velocity(left);
    right_motors.move_velocity(right);
}

inline void brake(void) {
    braking = true;
    left_motors.brake();
    right_motors.brake();
}

inline void set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}

void init(void);
} // namespace robot