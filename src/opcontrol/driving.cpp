#include "opcontrol/driving.h"
#include "robot.h"
#include "api.h"

#include <iostream>

namespace driving {
inline void leon_mode(int left_x, int left_y, int right_x, int right_y) {
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
		robot::velo(left_y, right_y);
	} else {
		robot::brake();
	}
}

void run() {
	while (true) {
		int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// swap out for other modes
		leon_mode(left_x, left_y, right_x, right_y);
		
		pros::delay(20);
	}
}
} // namespace driving