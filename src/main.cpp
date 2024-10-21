#include "main.h"
#include "essential.h"
#include "opcontrol/driving.h"
#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/movement.h"
#include "autonomous/movement/numerical_solvers.h"

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

	// auto start = pros::micros();
	// double sum = 0;
	// for (int i = 0; i < 1e4; i++) {
	// 	pathing::QuinticSpline sp;
	// 	sp.points.emplace_back(0, i);
	// 	sp.points.emplace_back(0, -i + 100);
	// 	sp.points.emplace_back(-50, 200);
	// 	sp.points.emplace_back(-sqrtf(i), 200);
	// 	sp.solve_coeffs(0, 0, 0, 0);
	// 	sum += sp.compute(0.5, 0)(0);
	// }
	// auto end = pros::micros();
	// printf("Time taken: %f\n", (end - start) / 1e6);
	// printf("Sum: %f\n", sum);

	pathing::QuinticSpline sp;
	sp.points.emplace_back(0, 0);
	sp.points.emplace_back(0, 100);
	sp.points.emplace_back(-50, 200);
	sp.points.emplace_back(-200, 200);
	sp.solve_coeffs(0, 0, 0, 0);
	std::cout << sp.debug_out() << std::endl;

	auto pos = Eigen::Vector2f(0, 0);
	float radius = 10;

	auto func = [=](float t) {
		return (sp.compute(t) - pos).norm() - radius;
	};
	auto deriv = [=](float t) {
		const auto rel = sp.compute(t) - pos;
		return rel.dot(sp.compute(t, 1) - pos) / rel.norm();
	};

	auto start = pros::millis();
	float sum = 0;
	for (int i = 0; i < 1e5; i++) {
		auto res = movement::solvers::newton(
			func, deriv, (float) i / 1e4, 0, 3, 10, 1e-9
		);
		sum += res.first;
	}
	auto end = pros::millis();
	printf("Time taken: %f\n", (end - start) / 1e3);
	printf("Sum: %f\n", sum);

	// auto res2 = movement::pure_pursuit::secant_intersect(
	// 	sp, Eigen::Vector2f(0, 0), 50, 0, 1, 0, 3, 15, 1e-1
	// );
	// printf("Secant intersect at %f with %f dist\n", res2.first, res2.second);
	// I know what you are

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