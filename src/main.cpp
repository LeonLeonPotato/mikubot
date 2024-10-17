#include "main.h"
#include "essential.h"
#include "opcontrol/driving.h"
#include "autonomous/odometry.h"
#include "autonomous/spline.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/strategy/test.h"

#include "api.h"

void initialize(void) {
	std::cout << "Initialize started" << std::endl;

	spline::init();
	robot::init();
	odometry::init();
	renderer::init();
	// spline::QuinticSpline sp;
	// sp.points.emplace_back(0, 0);
	// sp.points.emplace_back(0, 1);
	// sp.points.emplace_back(1, 1);
	// sp.points.emplace_back(1, 0);
	// sp.solve_coeffs(0, 0, 0, 0, 0, 0, 0, 0);
	// std::cout << sp.debug_out() << std::endl;

	// Eigen::Vector2f point(0.5, 0.5);
	// float t = pure_pursuit::compute_intersections(
	// 	sp, point, 0.7, 0.9, 0, 3, 15, 1e-1
	// );
	// std::cout << "test one computed: " << t << std::endl;

	// Eigen::VectorXf guess(10);
	// guess.setLinSpaced(10, 0, 1.5);
	// t = pure_pursuit::compute_intersections(
	// 	sp, point, 0.7, guess, 0, 3, 15, 1e-1
	// );
	// std::cout << "test two computed: " << t << std::endl;
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
	// competition_initialize();
	// autonrunner::init();
	std::cout << "Opcontrol started" << std::endl;
	test_strategy::run();
	driving::run();

	// auto targ = Eigen::Vector2f(50, 50);
	// while (true) {
	// 	float dtheta = robot::angular_diff(targ);
	// 	robot::velo((int) (dtheta * 100), (int) (-dtheta * 100));
	// 	printf("dtheta = %f, robot theta = %f\n", dtheta, robot::theta);

	// 	pros::delay(20);
	// }
	// test_strategy::run();
	
}