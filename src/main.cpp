#include "main.h"
#include "config.h"
#include "essential.h"
#include "ansicodes.h"
#include "gui/debugscreen.h"
#include "Eigen/Dense"
#include "opcontrol/impl/conveyor.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "telemetry.h" // IWYU pragma: keep

#include "autonomous/odometry.h"
#include "autonomous/strategies.h"

#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/funnymiku.h"
#include "gui/driverinfo.h"

#include "opcontrol/opcontrol.h"
#include <cmath>
#include <iostream>

#include "pros/apix.h" // IWYU pragma: keep

void initialize(void) {
	if (config::SIM_MODE) pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
	std::cout << PREFIX << "Initializing robot\n";

	robot::init();
	driverinfo::init();
	odometry::start_task();

	controls::conveyor::start_api_task();
	controls::wallmech::start_api_task();

	if (!pros::competition::is_connected()) {
		std::cout << PREFIX << "Robot is not connected to the field controller, manually calling functions\n";
		if (!config::SIM_MODE) {
			competition_initialize();
			debugscreen::init();
			autonomous();
		}
	}
}

void disabled(void) {
	std::cout << PREFIX << "Robot has been disabled\n";
}

void competition_initialize(void) {
	std::cout << PREFIX << "Competition initializing\n";

	autonselector::init();

	do {
		pros::delay(50);
	} while (autonselector::finished_selection == false);
	pros::delay(50);

	autonselector::destroy();

	std::cout << PREFIX << "Auton selection has finished\n";
}

void autonomous(void) {
	std::cout << PREFIX << "Running autonomous\n";
	// autonrunner::init();

	strategies::functions.at(strategies::chosen_strategy)();

	// autonrunner::destroy();
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	if (!config::SIM_MODE) autonrunner::destroy();
	if (!config::SIM_MODE) autonselector::destroy();

	while (true && !config::SIM_MODE) {
		for (auto& func : controls::ticks) {
			func();
		}
		pros::delay(10);
	}
}