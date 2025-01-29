#pragma once

#include "motor.h"

namespace hardware {
class DiffDrive {
    private:
        float track_width;
        float linear_mult;

        MotorGroup& left_motors;
        MotorGroup& right_motors;

        

    public:
        DiffDrive(MotorGroup& left_motors, MotorGroup& right_motors, float track_width, float linear_mult)
            : left_motors(left_motors), right_motors(right_motors),
            track_width(track_width), linear_mult(linear_mult) {}

        void unicycle(float linear, float angular, float linear_accel = 0, float angular_accel = 0) {
            float lv = linear + angular * track_width / 2.0f;
            float rv = linear - angular * track_width / 2.0f;
            float la = linear_accel + angular_accel * track_width / 2.0f;
            float ra = linear_accel - angular_accel * track_width / 2.0f;
            set_velocity(lv, rv, la, ra);
        }

        void set_velocity(float left, float right, float left_accel = 0, float right_accel = 0) {
            left_motors.set_desired_velocity(left, left_accel);
            right_motors.set_desired_velocity(right, right_accel);
        }

        void set_voltage(float left, float right) {
            left_motors.set_desired_voltage(left);
            right_motors.set_desired_voltage(right);
        }

        void set_brake_mode(BrakeMode mode) {
            left_motors.set_brake_mode(mode);
            right_motors.set_brake_mode(mode);
        }

        void brake(void) {
            left_motors.brake();
            right_motors.brake();
        }

        const MotorGroup& get_left_motors(void) const { return left_motors; }
        const MotorGroup& get_right_motors(void) const { return right_motors; }
};
} // namespace hardware