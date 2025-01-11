#include "essential.h"
#include "autonomous/controllers/pid.h"
#include "config.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.h"

using namespace robot;

static float left_kv = 0.0174021f*1000;
static float left_ka = 0.0028005359521f*1000;
static float left_kf = 0.852808*1000;
static controllers::PID left_pid(15.0f, 0.0, 0);

static float right_kv = 0.0174021f*1000;
static float right_ka = 0.0028153546704f*1000;
static float right_kf = 0.760359f*1000;
static controllers::PID right_pid(15.0f, 0.0, 0);

bool state::braking = false;
Eigen::Vector2f state::pos = Eigen::Vector2f::Zero();
Eigen::Vector2f state::velocity = Eigen::Vector2f::Zero();
Eigen::Vector2f state::acceleration = Eigen::Vector2f::Zero();
float state::theta = 0;
float state::angular_velocity = 0;
float state::angular_acceleration = 0;

int state::left_set_velocity = 0;
static float left_set_acceleration = 0.0f;
int state::right_set_velocity = 0;
static float right_set_acceleration = 0.0f;
int state::left_set_voltage = 0;
int state::right_set_voltage = 0;

char robot::match::team = 'R';
int robot::match::side = 1;

pros::Controller robot::master(pros::E_CONTROLLER_MASTER);
pros::Controller robot::partner(pros::E_CONTROLLER_PARTNER);

pros::adi::Pneumatics robot::doinker('b', false, true);
pros::adi::Pneumatics robot::ejector('c', false, false);
pros::adi::Pneumatics robot::clamp('a', false, false);

pros::Motor robot::conveyor(-17, pros::MotorGearset::blue);
pros::Motor robot::intake(-11);
pros::Motor robot::wallmech(1); 

pros::Imu robot::inertial(0);
pros::Optical robot::classifier(16);
pros::Rotation robot::side_encoder(0);
pros::Rotation robot::back_encoder(0);

// pros::MotorGroup robot::left_motors({-11, -12, -13}, pros::MotorGearset::blue);
// pros::MotorGroup robot::right_motors({1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup robot::left_motors({-8, -9, -10}, pros::MotorGearset::blue);
pros::MotorGroup robot::right_motors({18, 19, 20}, pros::MotorGearset::blue);

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
    left_set_voltage = (int) (std::clamp(left, -1.0f, 1.0f) * 12000.0f);
    right_set_voltage = (int) (std::clamp(right, -1.0f, 1.0f) * 12000.0f);
    braking = false;

    left_motors.move_voltage(left_set_voltage);
    right_motors.move_voltage(right_set_voltage);
}

void robot::volt(int left, int right) {
    robot::volt(left / 12000.0f, right / 12000.0f);
}

void robot::velo(float left, float right, float left_accel, float right_accel) {
    const int max = max_speed();

    float left_velo = (std::clamp(left, -1.0f, 1.0f) * max);
    float right_velo = (std::clamp(right, -1.0f, 1.0f) * max);
    left_set_velocity = (int) roundf(left_velo);
    right_set_velocity = (int) roundf(right_velo);

    left_set_acceleration = left_accel;
    right_set_acceleration = right_accel;

    braking = false;
}

void robot::brake(void) {
    braking = true;

    left_motors.brake();
    right_motors.brake();

    left_set_velocity = 0; left_set_acceleration = 0;
    right_set_velocity = 0; right_set_acceleration = 0;
    left_set_voltage = 0;
    right_set_voltage = 0;
}

void robot::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}

static float slew(float current, float target) {
    float max = fminf(current + 600, 12000);
    float min = fmaxf(current - 600, -12000);
    return std::clamp(target, min, max);
}

static void velocity_task(void* p) {
    const int max = max_speed();

    while (true) {
        if (!braking) {
            int left_sign = (left_set_velocity != 0) * (left_set_velocity > 0 ? 1 : -1);
            int right_sign = (right_set_velocity != 0) * (right_set_velocity > 0 ? 1 : -1);

            float left_ff = left_kv * left_set_velocity + left_ka * left_set_acceleration + left_kf * left_sign;
            float right_ff = right_kv * right_set_velocity + right_ka * right_set_acceleration + right_kf * right_sign;

            // printf("Left FF: %f | Right FF: %f\n", left_ff, right_ff);

            float left_fb = left_pid.get(left_set_velocity - left_motors.get_actual_velocity());
            float right_fb = right_pid.get(right_set_velocity - right_motors.get_actual_velocity());

            // printf("Left FB: %f | Right FB: %f\n", left_fb, right_fb);


            left_set_voltage = (int) slew(left_set_voltage, left_ff + left_fb);
            right_set_voltage = (int) slew(right_set_voltage, right_ff + right_fb);

            left_motors.move_voltage(left_set_voltage);
            right_motors.move_voltage(right_set_voltage);
        }

        pros::delay(10);
    }
}

void robot::init(void) {
    pros::c::task_create(velocity_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "velocity");

    inertial.reset(true);
    left_motors.set_brake_mode_all(config::default_brake_mode);
    right_motors.set_brake_mode_all(config::default_brake_mode);
    wallmech.set_brake_mode(pros::MotorBrake::hold);

    master.clear();
    pros::delay(150);
    partner.clear();
    pros::delay(150);

    master.set_text(0, 0, "Master");
    pros::delay(150);
    partner.set_text(0, 0, "Partner");
}