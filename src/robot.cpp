#include "robot.h"
#include "autonomous/strategies.h"

#include "api.h"

namespace robot {
char team;
strategies::Strategy auton_strategy;

bool braking = false;
double x = 0, velocity_x = 0, acceleration_x = 0;
double y = 0, velocity_y = 0, acceleration_y = 0;
double theta = 0, angular_velocity = 0, angular_acceleration = 0;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Imu inertial(20);
pros::Rotation side_encoder(-12);
pros::Rotation back_encoder(3);

pros::MotorGroup left_motors({-6, -7, -16});
pros::MotorGroup right_motors({11, 17, 15});

pros::MotorGroup intake({2, 9});
pros::Motor conveyor(10);

pros::adi::Pneumatics excluder('B', false);
pros::Optical classifier(13);

void init(void) {
    inertial.reset(true);
    left_motors.set_gearing(pros::E_MOTOR_GEARSET_06);
    right_motors.set_gearing(pros::E_MOTOR_GEARSET_06);
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
}
} // namespace robot