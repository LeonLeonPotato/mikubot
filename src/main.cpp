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
	odometry::init();
	renderer::init();
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
	strategies::test_strategy::run();
}

void opcontrol(void) {
	// competition_initialize();

	// autonomous();
	// pros::delay(100);

	std::cout << "Opcontrol started" << std::endl;

	int st = pros::millis();
	std::vector<std::pair<float, float>> vals;
	while (pros::millis() - st < 10000) {
		robot::volt(12000, -12000);
		vals.push_back({ robot::pos.x(), robot::pos.y() });
		pros::delay(20);
	}

	printf("Start\n");
	for (auto& val : vals) {
		std::cout << val.first << ", " << val.second << std::endl;
	}
	printf("End\n");

	for (auto& task : controls::start_tasks) {
		task();
	}

	//controls::lidartest::run();
}