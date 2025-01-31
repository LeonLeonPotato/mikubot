#include "main.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "config.h"
#include "essential.h"
#include "ansicodes.h"
#include "gui/debugscreen.h"
#include "Eigen/Dense"
#include "subsystems.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "telemetry.h" // IWYU pragma: keep

#include "autonomous/strategies.h"

#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/funnymiku.h"
#include "gui/driverinfo.h"

#include <cmath>
#include <iostream>

#include "pros/apix.h" // IWYU pragma: keep

void initialize(void) {
	if (config::SIM_MODE) pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
	std::cout << PREFIX << "Initializing robot\n";

	robot::init();
	driverinfo::init();
	// telemetry::start_task();

	for (auto& subsystem : subsystems::subsystems) {
		if (subsystem->has_api()) {
			subsystem->start_api_task();
		}
	}

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

void collect_pid_data(void) {
	robot::chassis.take_drive_mutex();

	std::vector<std::pair<float, float>> points;

	robot::velo(-0.5, 0.5);

	long long start = pros::micros();
	while ((pros::micros() - start) / 1e6f < 5) {
		float pos = robot::theta();
		points.push_back({(pros::micros() - start) / 1e6f, pos});
		pros::delay(10);
	}

	robot::velo(0, 0);

	std::cout << PREFIX << "Data collection finished\n";

	std::cout << "X = \\left[";
	for (int i = 0; i < points.size(); i++) {
		auto point = points[i];
		std::cout << "\\left(" << point.first << ",\\ " << point.second << "\\right)";
		if (i != points.size() - 1) {
			std::cout << ", ";
		}

		if (i % 5 == 0) {
			std::cout << "\n";
			std::cout.flush();
			pros::delay(100);
		}
	}

	std::cout << "\\right]\n";

	robot::chassis.give_drive_mutex();
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	if (!config::SIM_MODE) autonrunner::destroy();
	if (!config::SIM_MODE) autonselector::destroy();

	for (auto& subsystem : subsystems::subsystems) {
		subsystem->take_mutex();
	}

	while (true && !config::SIM_MODE) {
		for (auto& subsystem : subsystems::subsystems) {
			subsystem->tick();
		}
		pros::delay(10);
	}
}