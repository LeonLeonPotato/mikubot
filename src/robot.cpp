#include "robot.h"
#include "autonomous/strategies.h"

#include "api.h"

#include "nlohmann/json.h"

namespace robot {
char team;
strategies::Strategy auton_strategy;

inline namespace state {
bool braking = false;
double x = 0, velocity_x = 0, acceleration_x = 0;
double y = 0, velocity_y = 0, acceleration_y = 0;
double theta = 0, angular_velocity = 0, angular_acceleration = 0;
} // namespace state

namespace signatures {
const int blue_ring_id = 0;
const int red_ring_id = 1;
const int goal_id = 2;

pros::vision_signature_s_t blue_ring = pros::Vision::signature_from_utility(
    blue_ring_id, -3407, -3069, -3238, 9197, 10055, 9626, 4.300, 0
);
pros::vision_signature_s_t red_ring = pros::Vision::signature_from_utility(
    red_ring_id, 7489, 9515, 8502, 79, 509, 294, 3.000, 0
);
pros::vision_signature_s_t goal = pros::Vision::signature_from_utility(
    goal_id, -3265, -2831, -3048, -5487, -4767, -5127, 9.900, 0
);
} // namespace signatures

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Imu inertial(20);
pros::Rotation side_encoder(-12);
pros::Rotation back_encoder(3);

pros::MotorGroup left_motors({-6, -7, -16}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({11, 17, 15}, pros::MotorGearset::blue);

// pros::MotorGroup intake({2, 9});
pros::Motor conveyor(10);

pros::adi::Pneumatics excluder('B', false);
pros::Optical classifier(13);

pros::Vision vision(9);

void init(void) {
    inertial.reset(true);
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    vision.set_signature(signatures::blue_ring_id, &signatures::blue_ring);
    vision.set_signature(signatures::red_ring_id, &signatures::red_ring);
    vision.set_signature(signatures::goal_id, &signatures::goal);
}
} // namespace robot