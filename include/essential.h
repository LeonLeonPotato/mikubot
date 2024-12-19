#pragma once

#include "api.h"

#include "Eigen/Dense"

namespace robot {
constexpr float TRACKING_WHEEL_RADIUS = 4.1275f;
constexpr float DRIVETRAIN_LINEAR_MULT = 4.1275f * 0.6f;
constexpr float BACK_TRACKING_WHEEL_OFFSET = -6.46794f;
constexpr float SIDE_TRACKING_WHEEL_OFFSET = -3.7031f;

inline namespace state {
extern bool braking;
extern Eigen::Vector2f pos, velocity, acceleration;
extern float theta, angular_velocity, angular_acceleration;
extern int left_set_velocity, right_set_velocity;
extern int left_set_voltage, right_set_voltage;

inline const float angular_diff(const float desired, bool reversed = false) {
    const float res = fmod(desired - theta + (M_PI * (int) !reversed), M_TWOPI) - M_PI;
    return res + (res < -M_PI) * M_TWOPI;
}

inline const float angular_diff(const float desired_x, const float desired_y, const bool reversed = false) {
    return angular_diff(atan2(desired_x - pos.x(), desired_y - pos.y()), reversed);
}

inline const float angular_diff(const Eigen::Vector2f& point, const bool reversed = false) {
    return angular_diff(atan2(point(0) - pos.x(), point(1) - pos.y()), reversed);
}

inline const float distance(const float desired_x, const float desired_y) {
    return sqrtf((desired_x - pos.x()) * (desired_x - pos.x()) + (desired_y - pos.y()) * (desired_y - pos.y()));
}

inline const float distance(const Eigen::Vector2f& point) {
    return (point - pos).norm();
}
} // namespace state

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

namespace robot::match {
    extern char team;
    extern int side;

    inline std::string get_team_name(void) {
        return team == 'R' ? "Red" : "Blue";
    }

    inline std::string get_side_name(void) {
        return side == 1 ? "Left" : "Right";
    }
} // namespace state