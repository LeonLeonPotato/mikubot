#include "essential.h"

using namespace robot;

EngineMode state::engine_mode = EngineMode::DIRECT;

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
pros::Controller robot::partner(pros::E_CONTROLLER_PARTNER);

pros::adi::Pneumatics robot::doinker('b', false, true);
pros::adi::Pneumatics robot::ejector('c', false, false);
pros::adi::Pneumatics robot::clamp('d', false, false);

pros::Motor robot::conveyor(21, pros::MotorGearset::green);
pros::Motor robot::intake(-5);
pros::Motor robot::wallmech(-11);

pros::Imu robot::inertial(6);
pros::Optical robot::classifier(4);
pros::Rotation robot::side_encoder(0);
pros::Rotation robot::back_encoder(7);

pros::MotorGroup robot::left_motors({-8, 9, 10}, pros::MotorGearset::green);
pros::MotorGroup robot::right_motors({18, -19, -20}, pros::MotorGearset::green);

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

void robot::volt(float left, float right) {
    left = std::clamp(left, -1.0f, 1.0f) * 12000.0f;
    right = std::clamp(right, -1.0f, 1.0f) * 12000.0f;

    if (engine_mode == EngineMode::HIGH_TORQUE) {
        left *= 0.5f;
        right *= 0.5f;
    }

    if (engine_mode == EngineMode::HIGH_SPEED) {
        left *= 0.75f;
    }

    braking = false;
    left_motors.move_voltage((int) left);
    right_motors.move_voltage((int) right);
}

void robot::volt(int left, int right) {
    robot::volt(left / 12000.0f, right / 12000.0f);
}

void robot::velo(float left, float right) {
    int max = max_speed();
    float lv = std::clamp(left, -1.0f, 1.0f) * max;
    float rv = std::clamp(right, -1.0f, 1.0f) * max;
    braking = false;

    if (engine_mode == EngineMode::HIGH_TORQUE) {
        lv *= 0.5f;
        rv *= 0.5f;
    }

    if (engine_mode == EngineMode::HIGH_SPEED) {
        lv *= 0.75f;
    }

    left_motors.move_velocity((int) lv);
    right_motors.move_velocity((int) rv);
}

void robot::velo(int left, int right) {
    robot::velo(left / 12000.0f, right / 12000.0f);
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

    master.clear();
    pros::delay(200);
    partner.clear();
    pros::delay(200);
    master.set_text(0, 0, "Master");
    pros::delay(200);
    partner.set_text(0, 0, "Partner");
}