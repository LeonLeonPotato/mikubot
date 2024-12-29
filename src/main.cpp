#include "main.h"
#include "autonomous/pathing/base_path.h"
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
#include <iostream>
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
	constexpr int degree = 3;
	pathing::NthDegreeSpline<degree> path;

	for (int test = 0; test < 100; test++) {
		
		Eigen::Vector2f p2 = {rand() % 100, rand() % 100};
		std::vector<Eigen::Vector2f> points = {
			{rand() % 100, rand() % 100},
			{rand() % 100, rand() % 100},
			{rand() % 100, rand() % 100}
		};

		std::vector<pathing::Condition> natural;
		for (int i = 0; i < degree/2; i++) {
			natural.push_back(pathing::Condition::from_cartesian(i+2, 0, 0));
		}

		path.points = points;
		auto start_t = pros::micros();
		path.solve_coeffs(natural, natural);
		std::cout << PREFIX << "Solving took " << (pros::micros() - start_t) << "us\n";

		auto assert_point = [&](float t, int deriv, const Eigen::Vector2f& expected) {
			Eigen::Vector2f actual = path.compute(t, deriv);
			if ((actual - expected).norm() > 1e-2) {
				printf("%sExpected: (%f, %f) | Actual: (%f, %f) | t=%f, deriv=%d\n", PREFIX.c_str(), expected.x(), expected.y(), actual.x(), actual.y(), t, deriv);
				return 1;
			}
			return 0;
		};

		bool failed_c0_test = false;
		for (int t = 0; t < points.size(); t += 1) {
			failed_c0_test = failed_c0_test || assert_point((float) t - 1e-6, 0, points[t]);
			failed_c0_test = failed_c0_test || assert_point((float) t + 1e-6, 0, points[t]);
		}

		// bool failed_ic_test = assert_point(0, 2, {0, 0}) || assert_point(0, 3, {0, 0});
		// bool failed_bc_test = assert_point(path.maxt(), 2, {0, 0}) || assert_point(path.maxt(), 3, {0, 0});
		bool failed_ic_test = false;
		bool failed_bc_test = false;

		if (failed_c0_test) {
			std::cout << PREFIX << "Failed C0 test\n";
			std::cout << path.debug_out() << std::endl;
			break;
		}
		if (failed_ic_test) {
			std::cout << PREFIX << "Failed C1 test\n";
			std::cout << path.debug_out() << std::endl;
			break;
		}
		if (failed_bc_test) {
			std::cout << PREFIX << "Failed C2 test\n";
			std::cout << path.debug_out() << std::endl;
			break;
		}

		std::cout << PREFIX << "Test " << test << " passed\n";
	}
}

#define print_vec(v) std::cout << v.x() << ", " << v.y() << std::endl;

static void sample_spline(void) {
	constexpr int degree = 3;
	pathing::NthDegreeSpline<degree> path;

	std::vector<Eigen::Vector2f> points;
	for (int i = 0; i < 5; i++) {
		points.emplace_back(sinf(i)*i/2.0f, cosf(i)*i/2.0f);
		// points.emplace_back( ((i % 2)*2-1)*i, i);
	}

	path.points = points;
	auto start_t = pros::micros();
	path.solve_coeffs(path.natural_conditions, path.natural_conditions);
	std::cout << PREFIX << "Solving took " << (pros::micros() - start_t) << "us\n";

	print_vec(path.compute(4.15f, 0));
	print_vec(path.compute(4.15f, 1));
	print_vec(path.compute(4.15f, 2));
	print_vec(path.compute(4.15f, 3));

	std::cout << path.debug_out() << std::endl;
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	autonrunner::destroy(); pros::delay(10);
	autonselector::destroy(); pros::delay(10);
	// opcontrolfun::init();

	// test_cs();
	sample_spline();

	while (true) {
		for (auto& func : controls::ticks) {
			func();
		}
		pros::delay(10);
	}
}