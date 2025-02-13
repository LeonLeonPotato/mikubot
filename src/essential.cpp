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
hardware::Motor robot::conveyor {8, hardware::Gearset::BLUE, hardware::BrakeMode::BRAKE};
hardware::Motor robot::wallmech {6, hardware::Gearset::BLUE, hardware::BrakeMode::HOLD};
pros::Rotation robot::wallmech_encoder(2);

hardware::IMUGroup robot::inertial {{9, 10}};
pros::Rotation robot::horizontal_encoder(-19);
pros::Rotation robot::forwards_encoder(20);

hardware::MotorGroup robot::left_motors {
    {-1, 19, -20},
    hardware::Gearset::BLUE, 
    hardware::BrakeMode::COAST,
    {
        17.06875f, 2.8005f, 571.0516f, 
        {20.0f, 0.0f, 0.0f}
    },
    3000'000.0f
};
hardware::MotorGroup robot::right_motors {
    {15, 16, -17},
    hardware::Gearset::BLUE, 
    hardware::BrakeMode::COAST,
    {
        17.21498f, 2.8005f, 699.1533f,
        {20.0f, 0.0f, 0.0f}
    },
    3000'000.0f
};

hardware::DiffDriveChassis robot::chassis {
    left_motors, right_motors, inertial, forwards_encoder, horizontal_encoder,
    DRIVETRAIN_WIDTH, DRIVETRAIN_LINEAR_MULT, TRACKING_WHEEL_RADIUS,
    FORWARDS_TRACKING_WHEEL_OFFSET, HORIZONTAL_TRACKING_WHEEL_OFFSET
};

// Thank fuck Miniongolf on discord
extern "C" {
    void vexDeviceOpticalIntegrationTimeSet(void* device, double timeMs);
    void* vexDeviceGetByIndex(int32_t index);
}

void robot::init(void) {
    chassis.start_tracking_first_time();

    robot::classifier.set_led_pwm(255);
    vexDeviceOpticalIntegrationTimeSet(vexDeviceGetByIndex(robot::classifier.get_port()), 20);

    wallmech_encoder.set_position(0);

    master.clear();
    pros::delay(110);
    master.set_text(0, 0, "Master");
}