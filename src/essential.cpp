#include "essential.h"

using namespace robot;

EngineMode state::engine_mode = EngineMode::HIGH_SPEED;

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

const pros::motor_brake_mode_e_t config::default_brake_mode = pros::E_MOTOR_BRAKE_COAST;

pros::Controller robot::master(pros::E_CONTROLLER_MASTER);

pros::Imu robot::inertial(0);
pros::Rotation robot::side_encoder(0);
pros::Rotation robot::back_encoder(0);

pros::MotorGroup robot::left_motors({-1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup robot::right_motors({11, -12, -13}, pros::MotorGearset::blue);

void state::set_engine_mode(EngineMode mode) {
    if (mode == engine_mode) return;
    engine_mode = mode;

    left_motors.set_reversed(mode == EngineMode::HIGH_SPEED, 2);
    right_motors.set_reversed(mode != EngineMode::HIGH_SPEED, 2);
}

EngineMode state::get_engine_mode(void) {
    return engine_mode;
}

int robot::max_speed(void) {
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
}

void robot::volt(int left, int right) {
    left = fminf(fmaxf(left, -127), 127);
    right = fminf(fmaxf(right, -127), 127);
    braking = false;
    left_motors.move(left);
    right_motors.move(right);
}

void robot::velo(int left, int right) {
    int max = max_speed();
    int lv = (int) (std::clamp(left, -127, 127) / 127.0f * max);
    int rv = (int) (std::clamp(right, -127, 127) / 127.0f * max);
    braking = false;

    if (get_engine_mode() == EngineMode::HIGH_TORQUE) {
        lv = (int) (lv * 0.6);
        rv = (int) (rv * 0.6);
    }

    left_motors.move_velocity(lv);
    right_motors.move_velocity(rv);
}

void robot::velo(float left, float right) {
    int max = max_speed();
    int lv = (int) (std::clamp(left, -1.0f, 1.0f) * max);
    int rv = (int) (std::clamp(right, -1.0f, 1.0f) * max);
    braking = false;

    if (engine_mode == EngineMode::HIGH_TORQUE) {
        lv = (int) (lv * 0.6);
        rv = (int) (rv * 0.6);
    }

    left_motors.move_velocity(left);
    right_motors.move_velocity(right);
}

void robot::brake(void) {
    braking = true;
    left_motors.brake();
    right_motors.brake();
}

void robot::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}

void robot::init(void) {
    inertial.reset(true);
    left_motors.set_brake_mode_all(config::default_brake_mode);
    right_motors.set_brake_mode_all(config::default_brake_mode);

    // vision.set_signature(signatures::blue_ring_id, &signatures::blue_ring);
    // vision.set_signature(signatures::red_ring_id, &signatures::red_ring);
    // vision.set_signature(signatures::goal_id, &signatures::goal);
}