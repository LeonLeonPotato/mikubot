#include "opcontrol/impl/driving.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace controls;

struct Scaling {
	float dead_zone = 6.0f;
	float turn_scale = 1.0f;
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

static pros::task_t task;

void leon_mode(int left_x, int left_y, int right_x, int right_y) {
	if (abs(right_x) > 10 && abs(left_y) > 10) { // driving with turning
		int left = left_y + right_x;
		int right = left_y - right_x;
		robot::volt(left, right);
	} else if (abs(right_x) > 10 && abs(left_y) < 10) { // turning
		robot::volt(right_x, -right_x);
	} else if (abs(right_x) < 10 && abs(left_y) > 10) { // driving
		robot::volt(left_y, left_y);
	} else { // stop
		robot::brake();
	}
}

void leon_mode_velocity_based(int left_x, int left_y, int right_x, int right_y) {
	if (abs(right_x) > 10 && abs(left_y) > 10) { // driving with turning
		int left = left_y + right_x;
		int right = left_y - right_x;
		robot::velo(left, right);
	} else if (abs(right_x) > 10 && abs(left_y) < 10) { // turning
		robot::velo(right_x, -right_x);
	} else if (abs(right_x) < 10 && abs(left_y) > 10) { // driving
		robot::velo(left_y, left_y);
	} else { // stop
		robot::brake();
	}
}

void tank_drive(int left_x, int left_y, int right_x, int right_y) {
	if (std::min(abs(left_y), abs(right_y)) > 3) {
		robot::volt(left_y, right_y);
	} else {
		robot::brake();
	}
}

void tank_drive_velocity_based(int left_x, int left_y, int right_x, int right_y) {
	if (std::min(abs(left_y), abs(right_y)) > 3) {
		robot::velo(left_y, right_y);
	} else {
		robot::brake();
	}
}

void leon_mode_2(int left_x, int left_y, int right_x, int right_y) {
	// https://www.desmos.com/calculator/v88re8mjh5
	static Scaling left_scale;
	static Scaling right_scale = {.speed_scale = 3.0f, .smoothing = 0.75f};

	const float forward = left_scale.compute(left_y);
	const float turn = right_scale.compute(right_x);

	// printf("Driving left: %f | Right: %f\n", forward + turn, forward - turn);
	
	robot::volt(forward + turn, forward - turn);
}

void driving::tick() {
	const int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	const int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	const int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	const int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	leon_mode_2(left_x, left_y, right_x, right_y);
}

void driving::run() {
	while (true) {
		tick();
		pros::delay(20);
	}
}

static void local_run(void* p) {
	driving::run();
}

void driving::start_task() {
	task = pros::c::task_create(local_run, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void driving::stop_task() {
	pros::c::task_delete(task);
}