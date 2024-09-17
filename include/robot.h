#ifndef _PROS_ROBOT_H_
#define _PROS_ROBOT_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "api.h"

namespace robot {
const float TRACKING_WHEEL_RADIUS = 4.1275f;
const float BACK_TRACKING_WHEEL_OFFSET = 7.075f;
const float SIDE_TRACKING_WHEEL_OFFSET = 10.5f;

extern double x, velocity_x, acceleration_x;
extern double y, velocity_y, acceleration_y;
extern double theta, angular_velocity, angular_acceleration;

extern bool braking;

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

void init(void);
inline void velo(int left, int right) {
    braking = false;
    left_motors.move(left);
    right_motors.move(right);
}

inline void brake() {
    braking = true;
    left_motors.move(0);
    right_motors.move(0);
    left_motors.brake();
    right_motors.brake();
}
}

#endif