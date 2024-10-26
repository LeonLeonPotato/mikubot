#include "main.h"
#include "essential.h"
#include "opcontrol/driving.h"
#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/solvers.h"
#include "autonomous/strategies.h"

#include "api.h"

void initialize(void) {
	std::cout << "Initialize started" << std::endl;

	robot::init();
	// odometry::init();
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
}

#include "autonomous/future.h"

void opcontrol(void) {
	competition_initialize();
	std::cout << "Opcontrol started" << std::endl;
	// strategies::test_strategy::run();
	

	Future<int> f;

	auto dummy_task = [&f]() {
		pros::delay(1000);
		f.set_value(10);
	};

	pros::Task task(dummy_task);

	std::cout << f.get() << std::endl;
}