#include "main.h"
#include "essential.h"
#include "opcontrol/driving.h"
#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/solvers.h"

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

	pathing::QuinticSpline sp;
	sp.points.emplace_back(0, 0);
	sp.points.emplace_back(0, 100);
	sp.points.emplace_back(-50, 200);
	sp.points.emplace_back(-200, 200);
	sp.solve_coeffs(pathing::BaseParams {0, 0, 0, 0});
	std::cout << sp.debug_out() << std::endl;

	auto pos = Eigen::Vector2f(0, 0);
	float radius = 10;

	auto func = [=](float t) -> float {
		return (sp.compute(t) - pos).norm() - radius;
	};
	auto deriv = [=](float t) -> float {
		const Eigen::Vector2f rel = sp.compute(t) - pos;
		return rel.dot(sp.compute(t, 1)) / rel.norm();
	};
	auto vec_func = [=](Eigen::VectorXf& t) -> Eigen::VectorXf {
		return (sp.compute(t).colwise() - pos).colwise().norm().array() - radius;
	};
	auto vec_deriv = [=](Eigen::VectorXf& t) -> Eigen::VectorXf {
		const Eigen::Matrix2Xf rel = sp.compute(t).colwise() - pos;
		return rel.cwiseProduct(sp.compute(t, 1)).colwise().sum().cwiseQuotient(rel.colwise().norm());
	};

	auto start = pros::micros();
	float sum = 0;
	Eigen::VectorXf t0 = Eigen::VectorXf::LinSpaced(32, 0.05, sp.points.size() - 1.1);
	Eigen::VectorXf t1 = Eigen::VectorXf::LinSpaced(32, 0.1, sp.points.size() - 1.05);
	for (int i = 0; i < 1e4; i++) {
		auto res = solvers::newton_vec(
			vec_func, vec_deriv, t1, 0, sp.points.size() - 1, 5
		);
		sum += res.second;
	}
	auto end = pros::micros();
	printf("Time taken: %f\n", (end - start) / 1e6);
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