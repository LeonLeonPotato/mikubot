#include "opcontrol/driving.h"
#include "robot.h"
#include "api.h"

#include <iostream>
#include <math.h>

namespace driving {
pros::task_t driving_task;
const int driving_mode = 1;

void leon_mode(float right_x, float right_y, float left_x, float left_y) {
	if (fabsf(right_x) > 10 && fabsf(left_y) > 10) { // driving with turning
		float left = left_y + right_x;
		float right = left_y - right_x;
		robot::velo(left, right);
	} else if (fabsf(right_x) > 10 && fabsf(left_y) < 10) { // turning
		robot::velo(right_x, -right_x);
	} else if (fabsf(right_x) < 10 && fabs(left_y) > 10) { // driving
		robot::velo(left_y, left_y);
	} else { // stop
		robot::braking
		robot::left_motors.brake();
		right_wheels.stop(vex::brakeType::brake);
	}
}

void run(void* args) {
	void (*driving_modes[])(float, float, float, float) = {
		leon_mode
	};

	while (true) {
		int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		driving_modes[driving_mode - 1](right_x, right_y, left_x, left_y);
		
		pros::delay(20);
	}
}

void init(void) {
    driving_task = pros::Task::create(run);
}
}