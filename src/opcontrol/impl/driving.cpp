#include "opcontrol/impl/driving.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace controls;

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

inline void leon_mode_velocity_based(int left_x, int left_y, int right_x, int right_y) {
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

inline void tank_drive(int left_x, int left_y, int right_x, int right_y) {
	if (std::min(abs(left_y), abs(right_y)) > 3) {
		robot::volt(left_y, right_y);
	} else {
		robot::brake();
	}
}

inline void tank_drive_velocity_based(int left_x, int left_y, int right_x, int right_y) {
	if (std::min(abs(left_y), abs(right_y)) > 3) {
		robot::velo(left_y, right_y);
	} else {
		robot::brake();
	}
}

void driving::run() {
	while (true) {
		#ifndef MIKU_TESTENV
			int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
			int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
			int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
			int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		#else
			int left_x = 0;
			int left_y = 0;
			int right_x = 0;
			int right_y = 0;
		#endif

		// swap out for other modes
		leon_mode(left_x, left_y, right_x, right_y);

		pros::delay(20);
	}
}