#include "essential.h"
#include "autonomous/controllers/velocity.h"
#include "config.h"
#include "hardware/motor.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.h"

using namespace robot;

char robot::match::team = 'R';
int robot::match::side = 1;

pros::Controller robot::master(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics robot::doinker('b', false, false);
pros::adi::Pneumatics robot::ejector('c', false, false);
pros::adi::Pneumatics robot::clamp('a', false, false);

pros::Optical robot::classifier(1);
hardware::Motor robot::conveyor {4, hardware::Gearset::BLUE, hardware::BrakeMode::BRAKE};
hardware::Motor robot::wallmech {6, hardware::Gearset::BLUE, hardware::BrakeMode::HOLD};
pros::Rotation robot::wallmech_encoder(2);

hardware::IMUGroup robot::inertial {{18, 17}};
pros::Rotation robot::back_encoder(19);
pros::Rotation robot::side_encoder(20);

hardware::MotorGroup robot::left_motors {
    {1, -2, 3},
    hardware::Gearset::BLUE, 
    hardware::BrakeMode::COAST, 
    {
        17.4021f, 2.8005f, 0.0f, 
        {35.0f, 0.0f, 0.0f}
    },
    60000.0f
};
hardware::MotorGroup robot::right_motors {
    {-10, 9, -8},
    hardware::Gearset::BLUE, 
    hardware::BrakeMode::COAST,
    {
        17.4021f, 2.8005f, 0.0f,
        {35.0f, 0.0f, 0.0f}
    },
    60000.0f
};

hardware::DiffDriveChassis robot::chassis {
    left_motors, right_motors, inertial, side_encoder, back_encoder,
    DRIVETRAIN_WIDTH, DRIVETRAIN_LINEAR_MULT, TRACKING_WHEEL_RADIUS,
    LATERAL_TRACKING_WHEEL_OFFSET, HORIZONTAL_TRACKING_WHEEL_OFFSET
};

// Thank fuck Miniongolf on discord
extern "C" {
    void vexDeviceOpticalIntegrationTimeSet(void* device, double timeMs);
    void* vexDeviceGetByIndex(int32_t index);
}

void robot::init(void) {
    robot::classifier.set_led_pwm(255);
    vexDeviceOpticalIntegrationTimeSet(vexDeviceGetByIndex(robot::classifier.get_port()), 20);

    wallmech_encoder.set_position(0);

    master.clear();
    pros::delay(110);
    master.set_text(0, 0, "Master");
}