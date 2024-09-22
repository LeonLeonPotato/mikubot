#include "opcontrol/driving.h"
#include "robot.h"
#include "api.h"

#include <iostream>
#include <math.h>

namespace driving {
pros::task_t task;

inline void leon_mode(int right_x, int right_y, int left_x, int left_y) {
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

inline void differential_drive(int right_x, int right_y, int left_x, int left_y) {
	if (fmin(abs(left_x), abs(left_y)) > 3) {
		robot::velo(left_x, -left_x);
	} else {
		robot::brake();
	}
}

void run(void* args) {
	while (true) {
		int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		// swap out for other modes
		leon_mode(right_x, right_y, left_x, left_y);
		
		pros::delay(20);
	}
}

void init(void) {
    task = pros::c::task_create(run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "driving");
}

void start(void) {
    pros::c::task_suspend(task);
}

void stop(void) {
    pros::c::task_resume(task);
}
}