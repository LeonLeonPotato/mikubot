#pragma once

#include "imu.h"
#include "motor.h"
#include "odometry/twotracker.h"
#include "pros/rotation.hpp"

namespace hardware {
class DiffDriveChassis {
    private:
        float track_width;
        float linear_mult;
        float tracking_wheel_radius;
        float lateral_tracking_wheel_offset;
        float horizontal_tracking_wheel_offset;

        MotorGroup& left_motors;
        MotorGroup& right_motors;

        IMUGroup& imu;
        pros::Rotation& side_encoder;
        pros::Rotation& back_encoder;

        TwoTrackerOdometry odometry;

    public:
        DiffDriveChassis(
            MotorGroup& left_motors, MotorGroup& right_motors, 
            IMUGroup& imu, pros::Rotation& side_encoder, pros::Rotation& back_encoder,
            float track_width, float linear_mult, float tracking_wheel_radius,
            float lateral_tracking_wheel_offset, float horizontal_tracking_wheel_offset
        ) : left_motors(left_motors), right_motors(right_motors),
            imu(imu), side_encoder(side_encoder), back_encoder(back_encoder),
            track_width(track_width), linear_mult(linear_mult), tracking_wheel_radius(tracking_wheel_radius),
            lateral_tracking_wheel_offset(lateral_tracking_wheel_offset), horizontal_tracking_wheel_offset(horizontal_tracking_wheel_offset),
            odometry(Pose(0, 0, 0), track_width, linear_mult, tracking_wheel_radius, lateral_tracking_wheel_offset, horizontal_tracking_wheel_offset,
                left_motors, right_motors, imu, side_encoder, back_encoder) 
            {
                // imu.calibrate();
                // odometry.start_task();
            }

        bool take_drive_mutex(uint32_t timeout = TIMEOUT_MAX) {
            return left_motors.acquire_mutex(timeout) && right_motors.acquire_mutex(timeout);
        }
        void give_drive_mutex(void) {
            left_motors.release_mutex();
            right_motors.release_mutex();
        }
        bool poll_drive_mutex(void) const {
            return left_motors.poll_mutex() && right_motors.poll_mutex();
        }

        bool take_imu_mutex(uint32_t timeout = TIMEOUT_MAX) {
            return imu.acquire_mutex(timeout);
        }
        void give_imu_mutex(void) {
            imu.release_mutex();
        }

        void unicycle(float linear, float angular, float linear_accel = 0, float angular_accel = 0) {
            float lv = linear + angular * track_width / 2.0f;
            float rv = linear - angular * track_width / 2.0f;
            float la = linear_accel + angular_accel * track_width / 2.0f;
            float ra = linear_accel - angular_accel * track_width / 2.0f;
            set_velocity(lv, rv, la, ra);
        }

        void set_velocity(float left, float right, float left_accel = 0, float right_accel = 0) {
            float max_speed = this->max_speed();
            left_motors.set_desired_velocity(left * max_speed, left_accel);
            right_motors.set_desired_velocity(right * max_speed, right_accel);
        }

        void set_voltage(float left, float right) {
            left_motors.set_desired_voltage(left * 12000.0f);
            right_motors.set_desired_voltage(right * 12000.0f);
        }

        void set_brake_mode(BrakeMode mode) {
            left_motors.set_brake_mode(mode);
            right_motors.set_brake_mode(mode);
        }

        void brake(void) {
            left_motors.brake();
            right_motors.brake();
        }

        float max_speed(void) {
            return std::min(left_motors.get_max_speed(), right_motors.get_max_speed());
        }

        Pose get_pose(void) const { return odometry.get_pose(); }
        Eigen::Vector2f get_pos(void) const { return odometry.get_pose().get_pos(); }
        float x(void) const { return odometry.get_pose().get_pos().x(); }
        float y(void) const { return odometry.get_pose().get_pos().y(); }
        float theta(void) const { return odometry.get_pose().get_theta(); }
        float distance(const Pose& other) const { return odometry.get_pose().distance(other); }
        float distance(const Eigen::Vector2f& other) const { return odometry.get_pose().distance(other); }
        float distance(float x, float y) const { return odometry.get_pose().distance(x, y); }
        float angle_diff(float other) const { return odometry.get_pose().angle_diff(other); }
        float angle_diff(const Pose& other) const { return odometry.get_pose().angle_diff(other); }
        float angle_diff(const Eigen::Vector2f& other) const { return odometry.get_pose().angle_diff(other); }
        float angle_diff(float x, float y) const { return odometry.get_pose().angle_diff(x, y); }

        void set_pose(const Pose& pose) { odometry.set_pose(pose); }
        void tare_pose(void) { odometry.set_pose(0, 0, 0); }
        void set_pose(float x, float y, float theta) { odometry.set_pose(x, y, theta); }

        TwoTrackerOdometry& get_odometry(void) { return odometry; }

        const MotorGroup& get_left_motors(void) const { return left_motors; }
        const MotorGroup& get_right_motors(void) const { return right_motors; }
        const IMUGroup& get_imu(void) const { return imu; }
        const pros::Rotation& get_side_encoder(void) const { return side_encoder; }
        const pros::Rotation& get_back_encoder(void) const { return back_encoder; }
        float get_track_width(void) const { return track_width; }
        float get_linear_mult(void) const { return linear_mult; }
        float get_tracking_wheel_radius(void) const { return tracking_wheel_radius; }
        float get_lateral_tracking_wheel_offset(void) const { return lateral_tracking_wheel_offset; }
        float get_horizontal_tracking_wheel_offset(void) const { return horizontal_tracking_wheel_offset; }
};
} // namespace hardware