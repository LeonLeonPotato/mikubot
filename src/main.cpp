#include "main.h"
#include "essential.h"
#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/solvers.h"
#include "autonomous/strategies.h"
#include "gui/opcontrolinfo.h"
#include "opcontrol/opcontrol.h"
#include "telemetry.h"

#include "api.h"

void initialize(void) {
	std::cout << "Initialize started" << std::endl;

	robot::init();
	odometry::start_task();
	opcontrolinfo::init();

	// if (!pros::competition::is_connected()) {
	// 	std::cout << "Not connected to competition switch" << std::endl;
	// 	competition_initialize();
	// 	telemetry::start_task();
	// 	autonomous();
	// }
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
	autonrunner::init();

	strategies::functions.at(strategies::chosen_strategy)();
	autonrunner::destroy();
}

void opcontrol(void) {

	pathing::QuinticSpline qs;
	qs.points.emplace_back(0, 0);
	qs.points.emplace_back(0, 100);
	qs.points.emplace_back(70, 100);
	qs.set_relative(robot::pos);
	qs.solve_coeffs({
		.start_heading = 0,
		.start_magnitude = 10,
		.end_heading = 0,
		.end_magnitude = 0
	});
	long long start = pros::micros();
	qs.profile_path({
		.start_v = 10,
		.end_v = 0,
		.max_speed = 140,
		.accel = 300,
		.decel = 237,
		.track_width = 39,
		.ds = 0.1,
		.resolution = 5000
	});
	// asd
	printf("Profile path took %lld us\n", pros::micros() - start);

	std::cout << "[";
	int cnt = 0;
	for (auto& p : qs.get_profile()) {
		std::cout << "(" + std::to_string(p.s) + ", " + std::to_string(p.left_v) << ")";
		pros::delay(10);
		cnt++;
		if (cnt % 10 == 0) {
			std::cout << std::endl;
		}
		if (cnt != qs.get_profile().size()) {
			std::cout << ", ";
		}
	}
	std::cout << "]" << std::endl;

	std::cout << "Opcontrol started" << std::endl;

	for (auto& task : controls::start_tasks) {
		task();
	}
}