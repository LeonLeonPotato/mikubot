#include "main.h"
#include "essential.h"
#include "opcontrol/driving.h"
#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/strategy/test.h"

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
}

void opcontrol(void) {
	std::cout << "Opcontrol started" << std::endl;

	// pathing::QuinticSpline sp;
	// sp.points.emplace_back(0, 0);
	// sp.points.emplace_back(0, 100);
	// sp.points.emplace_back(-50, 200);
	// sp.points.emplace_back(-200, 200);
	// sp.solve_coeffs(0, 0, 0, 0);

	// std::cout << sp.debug_out() << std::endl;

	// Eigen::Vector2f test_1 = sp.compute(0.5, 0);
	// printf("Test 1: (%f, %f)\n", test_1(0), test_1(1));

	// Eigen::Vector2f test_2 = sp.compute(0.7, 1);
	// printf("Test 2: (%f, %f)\n", test_2(0), test_2(1));

	// Eigen::Vector2f test_3 = sp.compute(1.2, 4);
	// printf("Test 3: (%f, %f)\n", test_3(0), test_3(1));

	pathing::PolygonPath pp;
	pp.points.emplace_back(0, 0);
	pp.points.emplace_back(0, 100);
	pp.points.emplace_back(-50, 200);
	pp.points.emplace_back(-200, 200);

	// while (true) {
	// 	pros::vision_object_s_t obj = robot::vision.get_by_sig(0, robot::signatures::red_ring_id);
	// 	if (obj.signature == 0) {
	// 		robot::volt(0, 0);
	// 	} else {
	// 		int turn = obj.x_middle_coord - 158;
	// 		robot::volt(127 + turn, 127 - turn);
	// 	}
	// 	pros::delay(20);
	// }
	// test_strategy::run();
	
}