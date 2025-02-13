#pragma once

#include "hardware/odometry.h"
#include "hardware/motor.h"
#include "hardware/imu.h"
#include "mathtils.h"
#include "pros/rotation.hpp"

namespace hardware {
class HorizontalTrackerOdometry : public odometry::BaseOdometry {
    private:
        float track_width;
        float linear_mult;
        float tracking_wheel_radius;
        float forwards_tracking_wheel_offset;
        float horizontal_tracking_wheel_offset;

        const MotorGroup& left_motors;
        const MotorGroup& right_motors;

        const IMUGroup& imu;
        const pros::Rotation& side_encoder;
        const pros::Rotation& back_encoder;

    public:
        void run_task(void) override;

    public:
        HorizontalTrackerOdometry(const Pose& start_pose,
            float track_width, float linear_mult, float tracking_wheel_radius,
            float forwards_tracking_wheel_offset, float horizontal_tracking_wheel_offset,
            const MotorGroup& left_motors, const MotorGroup& right_motors,
            const IMUGroup& imu, const pros::Rotation& side_encoder, const pros::Rotation& back_encoder)
            : BaseOdometry(start_pose), 
            track_width(track_width), linear_mult(linear_mult), tracking_wheel_radius(tracking_wheel_radius),
            forwards_tracking_wheel_offset(forwards_tracking_wheel_offset), horizontal_tracking_wheel_offset(horizontal_tracking_wheel_offset),
            left_motors(left_motors), right_motors(right_motors),
            imu(imu), side_encoder(side_encoder), back_encoder(back_encoder) {}
};
}