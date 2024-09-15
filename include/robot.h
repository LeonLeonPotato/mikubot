#ifndef _PROS_ROBOT_H_
#define _PROS_ROBOT_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "api.h"

namespace robot {
extern double x, velocity_x, acceleration_x;
extern double y, velocity_y, acceleration_y;
extern double theta, angular_velocity, angular_acceleration;

extern pros::Controller master;
extern pros::IMU inertial;
extern pros::Rotation side_encoder;
extern pros::Rotation back_encoder;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

extern pros::MotorGroup intake;
extern pros::Motor conveyor;

extern pros::ADIPneumatics excluder;
extern pros::Optical classifier;

void init(void);
void velo(double left, double right);
void brake();
}

#endif