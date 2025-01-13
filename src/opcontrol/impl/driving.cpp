#include "opcontrol/impl/driving.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace controls;

static pros::task_t task = nullptr;

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

void leon_mode(int left_x, int left_y, int right_x, int right_y) {
	// https://www.desmos.com/calculator/v88re8mjh5
	static Scaling forward_scale;
	static Scaling turnging_scale = {.speed_scale = 0.6f, .smoothing = 0.0f};

	const float forward = forward_scale.compute(left_y);
	const float turn = turnging_scale.compute(right_x);
	
	robot::velo(forward + turn, forward - turn);
}

void driving::tick() {
	const int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	const int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	const int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	const int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	leon_mode(left_x, left_y, right_x, right_y);
}

void driving::run() {
	while (true) {
		driving::tick();

		pros::delay(10);
	}
}

static void local_run(void* p) {
	driving::run();
}

void driving::start_task() {
	if (task != nullptr) return;
	task = pros::c::task_create(local_run, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void driving::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void driving::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void driving::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}