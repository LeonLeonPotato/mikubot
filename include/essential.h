#pragma once

#include "api.h"

#include "Eigen/Dense"

namespace robot {
constexpr float TRACKING_WHEEL_RADIUS = 4.1275f;
constexpr float BACK_TRACKING_WHEEL_OFFSET = 0.018f;
constexpr float SIDE_TRACKING_WHEEL_OFFSET = -7.596f;

inline namespace state {
extern bool braking;
extern Eigen::Vector2f pos, velocity, acceleration;
extern double theta, angular_velocity, angular_acceleration;

inline float angular_diff(float desired, bool reversed = false) {
    return fmod(desired - theta + M_PI + (M_PI * (int) reversed), M_TWOPI) - M_PI;
}

inline float angular_diff(float desired_x, float desired_y, bool reversed = false) {
    return angular_diff(atan2(desired_x - pos.x(), desired_y - pos.y()), reversed);
}

inline float angular_diff(const Eigen::Vector2f& point, bool reversed = false) {
    return angular_diff(atan2(point(0) - pos.x(), point(1) - pos.y()), reversed);
}

inline float distance(float desired_x, float desired_y) {
    return sqrtf((desired_x - pos.x()) * (desired_x - pos.x()) + (desired_y - pos.y()) * (desired_y - pos.y()));
}

inline float distance(const Eigen::Vector2f& point) {
    return sqrtf((point(0) - pos.x()) * (point(0) - pos.x()) + (point(1) - pos.y()) * (point(1) - pos.y()));
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

#ifndef MIKU_TESTENV
    extern pros::Controller master;
    extern pros::Controller partner;

    extern pros::adi::Pneumatics doinker;
    extern pros::adi::Pneumatics ejector;
    extern pros::adi::Pneumatics clamp;

    extern pros::Motor conveyor;
    extern pros::Motor intake;
    extern pros::Motor wallmech;

    extern pros::IMU inertial;
    extern pros::Optical classifier;
    extern pros::Rotation side_encoder;
    extern pros::Rotation back_encoder;

    extern pros::MotorGroup left_motors;
    extern pros::MotorGroup right_motors;
#endif

int max_speed(void);

void volt(int left, int right);
void volt(float left, float right);
void velo(int left, int right);
void velo(float left, float right);

void brake(void);
void set_brake_mode(pros::motor_brake_mode_e_t mode);

void init(void);
} // namespace robot