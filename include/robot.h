#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "api.h"

#include "autonomous/strategies.h"

#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"

#define MULTIPLIER(T) (T == 'R' ? 1 : -1)

namespace robot {
const float TRACKING_WHEEL_RADIUS = 4.1275f;
const float BACK_TRACKING_WHEEL_OFFSET = 7.075f;
const float SIDE_TRACKING_WHEEL_OFFSET = 10.5f;

extern char team;
extern strategies::Strategy auton_strategy;

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

extern pros::Vision vision;

void init(void);
}