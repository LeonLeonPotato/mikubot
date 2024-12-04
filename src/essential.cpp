#include "essential.h"
#include "config.h"

using namespace robot;

bool state::braking = false;
Eigen::Vector2f state::pos = Eigen::Vector2f::Zero();
Eigen::Vector2f state::velocity = Eigen::Vector2f::Zero();
Eigen::Vector2f state::acceleration = Eigen::Vector2f::Zero();
float state::theta = 0;
float state::angular_velocity = 0;
float state::angular_acceleration = 0;

int state::left_set_velocity = 0;
int state::right_set_velocity = 0;
int state::left_set_voltage = 0;
int state::right_set_voltage = 0;

char robot::match::team = 'R';
int robot::match::side = 1;

#ifndef MIKU_TESTENV
    pros::Controller robot::master(pros::E_CONTROLLER_MASTER);
    pros::Controller robot::partner(pros::E_CONTROLLER_PARTNER);

    pros::adi::Pneumatics robot::doinker('b', false, true);
    pros::adi::Pneumatics robot::ejector('c', false, false);
    pros::adi::Pneumatics robot::clamp('d', false, false);

    pros::Motor robot::conveyor(10, pros::MotorGearset::green);
    pros::Motor robot::intake(-9);
    pros::Motor robot::wallmech(0);

    pros::Imu robot::inertial(19);
    pros::Optical robot::classifier(0);
    pros::Rotation robot::side_encoder(-20);
    pros::Rotation robot::back_encoder(-14);

    pros::MotorGroup robot::left_motors({-11, 12, -13}, pros::MotorGearset::blue);
    pros::MotorGroup robot::right_motors({1, -2, 4}, pros::MotorGearset::blue);
#endif

int robot::max_speed(void) {
    #ifndef MIKU_TESTENV
        switch (left_motors.get_gearing()) {
            case pros::MotorGears::blue:
                return 600;
                break;
            case pros::MotorGears::green:
                return 200;
                break;
            case pros::MotorGears::red:
                return 100;
                break;
            default:
                return 200;
                break;
        }
    #else
        return 1;
    #endif
}

void robot::volt(float left, float right) {
    left_set_voltage = (int) (std::clamp(left, -1.0f, 1.0f) * 12000.0f);
    right_set_voltage = (int) (std::clamp(right, -1.0f, 1.0f) * 12000.0f);
    braking = false;

    #ifndef MIKU_TESTENV
        left_motors.move_voltage(left_set_voltage);
        right_motors.move_voltage(right_set_voltage);
    #endif
}

void robot::volt(int left, int right) {
    robot::volt(left / 12000.0f, right / 12000.0f);
}

void robot::velo(float left, float right) {
    int max = max_speed();
    left_set_velocity = (int) (std::clamp(left, -1.0f, 1.0f) * max);
    right_set_velocity = (int) (std::clamp(right, -1.0f, 1.0f) * max);
    braking = false;

    #ifndef MIKU_TESTENV
        left_motors.move_velocity(left_set_velocity);
        right_motors.move_velocity(right_set_velocity);
    #endif
}

void robot::velo(int left, int right) {
    robot::velo(left / 12000.0f, right / 12000.0f);
}

void robot::brake(void) {
    braking = true;

    #ifndef MIKU_TESTENV
        left_motors.brake();
        right_motors.brake();
    #endif
}

void robot::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    #ifndef MIKU_TESTENV
        left_motors.set_brake_mode_all(mode);
        right_motors.set_brake_mode_all(mode);
    #endif
}

void robot::init(void) {
    #ifndef MIKU_TESTENV
        inertial.reset(true);
        left_motors.set_brake_mode_all(config::default_brake_mode);
        right_motors.set_brake_mode_all(config::default_brake_mode);

        master.clear();
        pros::delay(150);
        partner.clear();
        pros::delay(150);

        master.set_text(0, 0, "Master");
        pros::delay(150);
        partner.set_text(0, 0, "Partner");
    #endif
}