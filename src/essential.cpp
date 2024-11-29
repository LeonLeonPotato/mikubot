#include "essential.h"

using namespace robot;

bool state::braking = false;
Eigen::Vector2f state::pos = Eigen::Vector2f::Zero();
Eigen::Vector2f state::velocity = Eigen::Vector2f::Zero();
Eigen::Vector2f state::acceleration = Eigen::Vector2f::Zero();
double state::theta = 0;
double state::angular_velocity = 0;
double state::angular_acceleration = 0;

pros::vision_signature_s_t signatures::blue_ring = pros::Vision::signature_from_utility(
    signatures::blue_ring_id, -3407, -3069, -3238, 9197, 10055, 9626, 4.300, 0
);
pros::vision_signature_s_t signatures::red_ring = pros::Vision::signature_from_utility(
    signatures::red_ring_id, 7489, 9515, 8502, 79, 509, 294, 3.000, 0
);
pros::vision_signature_s_t signatures::goal = pros::Vision::signature_from_utility(
    signatures::goal_id, -3265, -2831, -3048, -5487, -4767, -5127, 9.900, 0
);

const pros::motor_brake_mode_e_t config::default_brake_mode 
    = pros::E_MOTOR_BRAKE_COAST;

#ifndef MIKU_TESTENV
    pros::Controller robot::master(pros::E_CONTROLLER_MASTER);
    pros::Controller robot::partner(pros::E_CONTROLLER_PARTNER);

    pros::adi::Pneumatics robot::doinker('b', false, true);
    pros::adi::Pneumatics robot::ejector('c', false, false);
    pros::adi::Pneumatics robot::clamp('d', false, false);

    pros::Motor robot::conveyor(21, pros::MotorGearset::green);
    pros::Motor robot::intake(-9);
    pros::Motor robot::wallmech(-11);

    pros::Imu robot::inertial(7);
    pros::Optical robot::classifier(0);
    pros::Rotation robot::side_encoder(4);
    pros::Rotation robot::back_encoder(5);

    pros::MotorGroup robot::left_motors({1, -2, -3}, pros::MotorGearset::blue);
    pros::MotorGroup robot::right_motors({11, 12, -13}, pros::MotorGearset::blue);
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
        return 100;
    #endif
}

void robot::volt(float left, float right) {
    left = std::clamp(left, -1.0f, 1.0f) * 12000.0f;
    right = std::clamp(right, -1.0f, 1.0f) * 12000.0f;

    braking = false;

    #ifndef MIKU_TESTENV
        left_motors.move_voltage((int) left);
        right_motors.move_voltage((int) right);
    #endif
}

void robot::volt(int left, int right) {
    robot::volt(left / 12000.0f, right / 12000.0f);
}

void robot::velo(float left, float right) {
    int max = max_speed();
    left = std::clamp(left, -1.0f, 1.0f) * max;
    right = std::clamp(right, -1.0f, 1.0f) * max;
    braking = false;

    #ifndef MIKU_TESTENV
        left_motors.move_velocity((int) left);
        right_motors.move_velocity((int) right);
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

        // vision.set_signature(signatures::blue_ring_id, &signatures::blue_ring);
        // vision.set_signature(signatures::red_ring_id, &signatures::red_ring);
        // vision.set_signature(signatures::goal_id, &signatures::goal);

        master.clear();
        pros::delay(150);
        partner.clear();
        pros::delay(150);
        master.set_text(0, 0, "Master");
        pros::delay(150);
        partner.set_text(0, 0, "Partner");
    #endif
}