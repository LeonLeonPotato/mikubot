#include "main.h"
#include "autonomous/pathing/cubic_spline.h"
#include "essential.h"
#include "ansicodes.h"
#include "opcontrol/test/odom_center.h"
#include "Eigen/Dense"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "telemetry.h" // IWYU pragma: keep

#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "autonomous/strategies.h"
#include "autonomous/pathing/polynomial.h"

#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/goofymiku.h"
#include "gui/opcontrolinfo.h"

#include "opcontrol/opcontrol.h"
#include <cmath>
#include <string>

#include "pros/apix.h"

void initialize(void) {
	// ONLY uncomment for simulator usage!!!
	// pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
	std::cout << PREFIX << "Initializing robot\n";

	robot::init();
	odometry::start_task();
	opcontrolinfo::init();

	if (!pros::competition::is_connected()) {
		std::cout << PREFIX << "Robot is not connected to the field controller, manually calling functions\n";
		// competition_initialize();
		// telemetry::start_task();
		// autonomous();
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
	autonselector::destroy();
	std::cout << PREFIX << "Auton selection has finished\n";

	autonrunner::init();
}

void autonomous(void) {
	std::cout << PREFIX << "Running autonomous\n";
	autonrunner::init();

	strategies::functions.at(strategies::chosen_strategy)();
}

static void test_cs(void) {
	pathing::CubicSpline cb;
	cb.points = {
		{0, 0},
		{9, 4},
	};

	cb.solve_coeffs(pathing::BaseParams {0, 1, 0, -1});

	std::cout << cb.debug_out() << std::endl;
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	autonrunner::destroy(); pros::delay(10);
	autonselector::destroy(); pros::delay(10);
	// opcontrolfun::init();

	test_cs();

	while (true) {
		for (auto& func : controls::ticks) {
			func();
		}
		pros::delay(10);
	}
}