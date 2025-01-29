#include "essential.h"
#include "autonomous/controllers/velocity.h"
#include "config.h"
#include "hardware/motor.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.h"

using namespace robot;

controllers::VelocityController robot::left_velo_controller({
    17.4021f, 2.8005f, 852.808f, 
    {0.0f, 0.0f, 0.0f}
});
controllers::VelocityController robot::right_velo_controller({
    17.4021f, 2.8005f, 760.359f,
    {0.0f, 0.0f, 0.0f}
});

bool state::braking = false;
Eigen::Vector2f state::pos = Eigen::Vector2f::Zero();
Eigen::Vector2f state::velocity = Eigen::Vector2f::Zero();
Eigen::Vector2f state::acceleration = Eigen::Vector2f::Zero();
float state::theta = 0;
float state::angular_velocity = 0;
float state::angular_acceleration = 0;

int state::left_set_velocity = 0; static float left_set_acceleration = 0.0f;
int state::right_set_velocity = 0; static float right_set_acceleration = 0.0f;
int state::left_set_voltage = 0;
int state::right_set_voltage = 0;

char robot::match::team = 'R';
int robot::match::side = 1;

pros::Controller robot::master(pros::E_CONTROLLER_MASTER);
pros::Controller robot::partner(pros::E_CONTROLLER_PARTNER);

pros::adi::Pneumatics robot::doinker('b', false, false);
pros::adi::Pneumatics robot::ejector('c', false, false);
pros::adi::Pneumatics robot::clamp('a', false, false);

pros::Optical robot::classifier(0);
hardware::Motor robot::conveyor(0, hardware::Gearset::BLUE, hardware::BrakeMode::BRAKE);
hardware::Motor robot::intake(0, hardware::Gearset::BLUE, hardware::BrakeMode::HOLD);
hardware::Motor robot::wallmech(0, hardware::Gearset::BLUE, hardware::BrakeMode::HOLD);
pros::Rotation robot::wallmech_encoder(0);

pros::Imu robot::inertial(18);
pros::Rotation robot::back_encoder(19);
pros::Rotation robot::side_encoder(20);

// pros::MotorGroup robot::left_motors({-11, -12, -13}, pros::MotorGearset::blue);
// pros::MotorGroup robot::right_motors({1, 2, 3}, pros::MotorGearset::blue);
hardware::MotorGroup robot::left_motors({1, -2, 3}, 
    hardware::Gearset::BLUE, 
    hardware::BrakeMode::COAST, 
    controllers::VelocityControllerArgs {
        17.4021f, 2.8005f, 852.808f, 
        {30.0f, 0.0f, 0.0f}});
hardware::MotorGroup robot::right_motors({-10, 9, -8}, 
    hardware::Gearset::BLUE, 
    hardware::BrakeMode::COAST,
    controllers::VelocityControllerArgs {
        17.4021f, 2.8005f, 760.359f,
        {30.0f, 0.0f, 0.0f}});

void robot::volt(float left, float right) {
    left_motors.set_desired_voltage(left);
    right_motors.set_desired_voltage(right);
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

static float average(std::vector<double> vec) {
    double sum = 0;
    for (auto& val : vec) {
        sum += val;
    }
    return sum / vec.size();
}

static void velocity_task(void* p) {
    const int max = max_speed();

    while (true) {
        if (!braking) {
            float left_current = (config::ONLY_BRAIN || config::SIM_MODE) ? left_set_velocity : average(left_motors.get_actual_velocity_all());
            float right_current = (config::ONLY_BRAIN || config::SIM_MODE) ? right_set_velocity : average(right_motors.get_actual_velocity_all());
            
            float left_target_voltage = robot::left_velo_controller.get(
                left_current, left_set_velocity, left_set_acceleration);
            float right_target_voltage = robot::right_velo_controller.get(
                right_current, right_set_velocity, right_set_acceleration);

            left_set_voltage = (int) slew(left_set_voltage, left_target_voltage);
            right_set_voltage = (int) slew(right_set_voltage, right_target_voltage);

            left_motors.move_voltage(left_set_voltage);
            right_motors.move_voltage(right_set_voltage);
        }

        pros::delay(10);
    }
}

void robot::init(void) {
    pros::c::task_create(velocity_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "velocity");

    if (!config::SIM_MODE) {
        robot::classifier.set_led_pwm(255);

        inertial.reset(true);
        left_motors.set_brake_mode_all(config::default_brake_mode);
        right_motors.set_brake_mode_all(config::default_brake_mode);
        wallmech.set_brake_mode(pros::MotorBrake::hold);
        wallmech_encoder.set_position(0);

        master.clear();
        pros::delay(150);
        partner.clear();
        pros::delay(150);

        master.set_text(0, 0, "Master");
        pros::delay(150);
        partner.set_text(0, 0, "Partner");
    }
}