#include "essential.h"
#include "autonomous/autonconfig.h"

#include "api.h"

namespace robot {
inline namespace state {
bool braking = false;
double x = 0, velocity_x = 0, acceleration_x = 0;
double y = 0, velocity_y = 0, acceleration_y = 0;
double theta = 0, angular_velocity = 0, angular_acceleration = 0;
} // namespace state

namespace signatures {
const int blue_ring_id = 1;
const int red_ring_id = 2;
const int goal_id = 3;
const int test_id = 4;

pros::vision_signature_s_t blue_ring = pros::Vision::signature_from_utility(
    blue_ring_id, -3407, -3069, -3238, 9197, 10055, 9626, 0.000, 0
);
pros::vision_signature_s_t red_ring = pros::Vision::signature_from_utility(
    red_ring_id, 7489, 9515, 8502, 79, 509, 294, 0.000, 0
);
pros::vision_signature_s_t goal = pros::Vision::signature_from_utility(
    goal_id, -3265, -2831, -3048, -5487, -4767, -5127, 9.900, 0
);
pros::vision_signature_s_t test = pros::Vision::signature_from_utility(
    test_id, 9891, 11591, 10740, -665, 195, -236, 3, 0
);
} // namespace signatures

pros::Vision vision(12);
pros::Motor motor(-5);

void init(void) {
    vision.set_signature(signatures::blue_ring_id, &signatures::blue_ring);
    vision.set_signature(signatures::red_ring_id, &signatures::red_ring);
    vision.set_signature(signatures::goal_id, &signatures::goal);
    vision.set_signature(signatures::test_id, &signatures::test);
    // vision.set_exposure(150);
}
} // namespace robot