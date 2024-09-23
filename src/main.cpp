#include "main.h"
#include "robot.h"
#include "opcontrol/driving.h"
#include "autonomous/odometry.h"
#include "autonomous/spline.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"

#include "api.h"

#include <iostream>

void initialize(void) {
	std::cout << "Initialize started" << std::endl;

	spline::init();
	robot::init();
	odometry::init();
}

void disabled(void) {
	std::cout << "Robot has been disabled" << std::endl;
}

void competition_initialize(void) {
	std::cout << "Comp init started" << std::endl;

	autonselector::init();
	while (autonselector::finished_selection == false) {
		pros::delay(20);
	}
	autonselector::destroy();
	std::cout << "Auton selector finished" << std::endl;
}

void autonomous(void) {
	std::cout << "Auton started" << std::endl;
}

void opcontrol(void) {
	std::cout << "Opcontrol started" << std::endl;
	driving::run();
}