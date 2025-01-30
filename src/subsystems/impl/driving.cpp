#include "subsystems/impl/driving.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace subsystems;

Driving* Driving::instance = nullptr;

struct Scaling {
	float dead_zone = 5.0f;
	float speed_scale = 1.0f;
	float smoothing = 0.45f;
	float max_speed = 1.0f;

	const float compute(int input) const {
		const float raw = (2 * (int) (input > 0) - 1) // sign
			* ((float) (abs(input) > dead_zone)) // dead zone
			* (1 - smoothing) * (abs(input) * speed_scale / 127.0f) // numerator
			/ (1 + smoothing * (1 - 2 * abs(input) / 127.0f)); // denominator

		return std::clamp(raw, -max_speed, max_speed);
	}
};

static void leon_mode(int left_x, int left_y, int right_x, int right_y) {
	// https://www.desmos.com/calculator/v88re8mjh5
	static Scaling forward_scale;
	static Scaling turnging_scale = {.speed_scale = 0.6f, .smoothing = 0.4f};

	const float forward = forward_scale.compute(left_y);
	const float turn = turnging_scale.compute(right_x);
	
	robot::velo(forward - turn, forward + turn);
}

void Driving::tick(void) {
	if (!poll_mutex()) return;
	if (!robot::chassis.poll_drive_mutex()) {
		robot::chassis.take_drive_mutex();
	}

	const int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	const int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	const int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	const int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	leon_mode(left_x, left_y, right_x, right_y);
}