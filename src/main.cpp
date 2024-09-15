#include "main.h"
#include "api.h"

#include "robot.h"
#include <math.h>

void initialize() {
	robot::init();
	fmin(2, 2);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		pros::delay(250);
	}
}