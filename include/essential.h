#pragma once

#include "api.h"

#include "Eigen/Dense"
#include "autonomous/controllers/velocity.h"
#include "hardware/chassis.h"
#include "hardware/imu.h"
#include "hardware/motor.h"

namespace robot {
constexpr float TRACKING_WHEEL_RADIUS = 2.54f;
constexpr float DRIVETRAIN_LINEAR_MULT = 4.1275f * 0.75f;
constexpr float LATERAL_TRACKING_WHEEL_OFFSET = -6.46794f;
constexpr float HORIZONTAL_TRACKING_WHEEL_OFFSET = -3.7031f;
constexpr float DRIVETRAIN_WIDTH = 30.29204f;

extern pros::Controller master;

extern pros::adi::Pneumatics doinker;
extern pros::adi::Pneumatics ejector;
extern pros::adi::Pneumatics clamp;

extern pros::Optical classifier;
extern hardware::Motor conveyor;
extern hardware::Motor intake;
extern hardware::Motor wallmech;
extern pros::Rotation wallmech_encoder;

extern hardware::IMUGroup inertial;
extern pros::Rotation side_encoder;
extern pros::Rotation back_encoder;

extern hardware::MotorGroup left_motors;
extern hardware::MotorGroup right_motors;

extern hardware::DiffDriveChassis chassis;

inline float angular_diff(const float desired, bool reversed = false) {
    const float res = fmodf(desired - chassis.theta() + (M_PI * (int) !reversed), M_TWOPI) - M_PI;
    return res + (res < -M_PI) * M_TWOPI;
}

inline float angular_diff(const float desired_x, const float desired_y, const bool reversed = false) {
    return angular_diff(atan2f(desired_x - chassis.x(), desired_y - chassis.y()), reversed);
}

inline float angular_diff(const Eigen::Vector2f& point, const bool reversed = false) {
    return angular_diff(atan2f(point(0) - chassis.x(), point(1) - chassis.y()), reversed);
}

inline float distance(const float desired_x, const float desired_y) {
    return sqrtf((desired_x - chassis.x()) * (desired_x - chassis.x()) + (desired_y - chassis.y()) * (desired_y - chassis.y()));
}

inline float distance(const Eigen::Vector2f& point) {
    return chassis.distance(point);
}

inline float distance(const Pose& pose) {
    return chassis.distance(pose);
}

inline float max_speed(void) { return chassis.max_speed(); }

inline Pose get_pose(void) { return chassis.get_pose(); }
inline Eigen::Vector2f get_pos(void) { return chassis.get_pos(); }
inline Eigen::Vector2f pos(void) { return chassis.get_pos(); }
inline float x(void) { return chassis.x(); }
inline float y(void) { return chassis.y(); }
inline float theta(void) { return chassis.theta(); }

inline void volt(int left, int right) { volt(left / 12000.0f, right / 12000.0f); }
inline void volt(float left, float right) { chassis.set_voltage(left, right); }
inline void velo(float left, float right, float left_accel = 0.0f, float right_accel = 0.0f) { chassis.set_velocity(left, right, left_accel, right_accel); }
inline void unicycle(float linear, float angular, float linear_accel = 0.0f, float angular_accel = 0.0f) { chassis.unicycle(linear, angular, linear_accel, angular_accel); }
inline void brake(void) { chassis.brake(); }

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
} // namespace robot::match