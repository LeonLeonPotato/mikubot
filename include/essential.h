#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "api.h"
#include "autonomous/autonconfig.h"

#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"

namespace robot {
const float TRACKING_WHEEL_RADIUS = 4.1275f;
const float BACK_TRACKING_WHEEL_OFFSET = 7.075f;
const float SIDE_TRACKING_WHEEL_OFFSET = 10.5f;

inline namespace state {
    extern bool braking;
    extern double x, velocity_x, acceleration_x;
    extern double y, velocity_y, acceleration_y;
    extern double theta, angular_velocity, angular_acceleration;

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

    inline float angular_diff(Eigen::Vector2f& point) {
        return angular_diff(atan2(point(0) - x, point(1) - y));
    }

    inline float distance(float desired_x, float desired_y) {
        return sqrtf((desired_x - x) * (desired_x - x) + (desired_y - y) * (desired_y - y));
    }

    inline float distance(Eigen::Vector2f& point) {
        return sqrtf((point(0) - x) * (point(0) - x) + (point(1) - y) * (point(1) - y));
    }
} // namespace state

namespace signatures {
    extern const int blue_ring_id;
    extern const int red_ring_id;
    extern const int goal_id;

    extern pros::vision_signature_s_t blue_ring;
    extern pros::vision_signature_s_t red_ring;
    extern pros::vision_signature_s_t goal;
} // namespace signatures

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

inline void velo(int left, int right) {
    left = std::clamp(left, -127, 127);
    right = std::clamp(right, -127, 127);
    braking = false;
    left_motors.move(left);
    right_motors.move(right);
}

inline void brake(void) {
    braking = true;
    left_motors.move(0);
    right_motors.move(0);
    left_motors.brake();
    right_motors.brake();
}

inline void set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}

void init(void);
}