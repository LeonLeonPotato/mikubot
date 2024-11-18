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

void driving::tick() {

}

void driving::run() {
	float left_last_velo = robot::left_motors.get_actual_velocity();
	float right_last_velo = robot::right_motors.get_actual_velocity();

	while (true) {
		int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		int engine_mode = robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) 
			+ robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

		switch (engine_mode) {
			case 0:
				robot::set_engine_mode(robot::EngineMode::DIRECT);
				break;
			case 1:
				robot::set_engine_mode(robot::EngineMode::HIGH_SPEED);
				break;
			case 2:
				robot::set_engine_mode(robot::EngineMode::HIGH_TORQUE);
				break;
		}

		leon_mode_velocity_based(left_x, left_y, right_x, right_y);
		it++;

		long long dt = pros::micros() - cur_time;
		cur_time = pros::micros();

		pros::delay(20);
	}
}