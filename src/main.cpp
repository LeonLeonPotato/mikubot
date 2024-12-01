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

#include "api.h"

void initialize(void) {
	std::cout << "Initialize started" << std::endl;

	robot::init();
	odometry::start_task();
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
}

void opcontrol(void) {
	//competition_initialize();

	// robot::pos = Eigen::Vector2f(1, 1);
	// robot::theta = 0;
	// printf("Test 1: %f\n", robot::angular_diff(Eigen::Vector2f(1, 0), false));
	// printf("Test 2: %f\n", robot::angular_diff(Eigen::Vector2f(2, 1), false));

	autonomous();
	pros::delay(100);

	std::cout << "Opcontrol started" << std::endl;

	for (auto& task : controls::start_tasks) {
		task();
	}

	//controls::lidartest::run();
}