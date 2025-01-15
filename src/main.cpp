#include "main.h"
#include "autonomous/controllers/pid.h"
#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/strategy/test.h"
#include "config.h"
#include "essential.h"
#include "ansicodes.h"
#include "gui/debugscreen.h"
#include "opcontrol/test/odom_center.h"
#include "Eigen/Dense"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "telemetry.h" // IWYU pragma: keep
#include "gui/simtest.h"

#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "autonomous/strategies.h"
#include "autonomous/pathing/polynomial.h"

#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/funnymiku.h"
#include "gui/driverinfo.h"

#include "opcontrol/opcontrol.h"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "pros/apix.h" // IWYU pragma: keep

void initialize(void) {
	if (config::SIM_MODE) pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
	std::cout << PREFIX << "Initializing robot\n";

	robot::init();
	driverinfo::init();
	odometry::start_task();

	if (!pros::competition::is_connected()) {
		std::cout << PREFIX << "Robot is not connected to the field controller, manually calling functions\n";
		// simtest::init();
		if (!config::SIM_MODE) competition_initialize();
		if (!config::SIM_MODE) debugscreen::init();
		// telemetry::start_task();
		if (!config::SIM_MODE) autonomous();
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
}

void autonomous(void) {
	std::cout << PREFIX << "Running autonomous\n";
	// autonrunner::init();

	strategies::functions.at(strategies::chosen_strategy)();

	// autonrunner::destroy();
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

static void unit_test_boomerang(void) {
	using Vec = const Eigen::Vector2f;
	using PID = controllers::PID;
	auto test_tick = [&] (Vec Rpos, float theta, Vec dest, float lead) {
		robot::pos = Rpos;
		PID linear(1, 0, 0); PID angular(1, 0, 0); 
		auto res = movement::simple::boomerang_tick(dest, theta, lead, {}, {linear, angular});
		std::cout << res.debug_out() << std::endl;
	};

	auto test_standard = [&] (Vec Rpos, float theta, Vec dest, float lead) {
		robot::pos = Rpos;
		PID linear(1, 0, 0); PID angular(1, 0, 0); 
		movement::SimpleMovementParams params {.linear_exit_threshold=0.1f, .timeout=2000, .delay=200};
		auto res = movement::simple::boomerang_async(dest, theta, lead, params, {linear, angular});
		std::cout << res.get().debug_out() << std::endl;
	};


	test_tick({0, 0}, M_PI, {1, 1}, 0.5f);
	test_tick({0, 0}, 0, {1, 1}, 0.5f);

	test_standard({0, 0}, M_PI/2, {2, 1}, 0.3f);
	std::cout << "Test done" << std::endl;
}

static void print_vector(std::vector<float>& vec, std::string name) {
	printf("%s = \\left[", name.c_str());
	for (int i = 0; i < vec.size(); i++) {
		printf("\\left(%f,\\ %f\\right)", i*0.02, vec[i]);
		if (i != vec.size() - 1) printf(",\\ ");
	}
	printf("\\right]\n");
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	if (!config::SIM_MODE) autonrunner::destroy();
	if (!config::SIM_MODE) autonselector::destroy();
	// opcontrolfun::init();

	// test_cs();
	// sample_spline();
	// unit_test_boomerang();
	// int dir = 1;

	// while (true) {
	// 	robot::velo(0, 1);
	// 	pros::delay(20);
	// 	printf("pos: (%f, %f), theta: %f\n", robot::pos.x(), robot::pos.y(), robot::theta);
	// }

	// auto unit_test_angle = [&] (float desired, float theta = 1.0f, bool reversed = false) {
	// 	const float res = fmodf(desired - theta + (M_PI * (int) !reversed), M_TWOPI) - M_PI;
	// 	return (res + (res < -M_PI) * M_TWOPI) * (reversed ? 1 : -1);
	// };

	// auto unit_test_point = [&] (const Eigen::Vector2f& point, float theta = 0.0f, bool reversed = false) {
	// 	return unit_test_angle(atan2f(point(0) - 0, point(1) - 0), theta, reversed);
	// };

	// for (float angle = -100; angle <= 400; angle += 20) {
	// 	float rad = rad(angle);
	// 	Eigen::Vector2f projected = Eigen::Vector2f {sinf(rad), cosf(rad)};
	// 	printf("%sAngle: %f | Result: %f\n", PREFIX.c_str(), deg(rad), deg(unit_test_angle(rad, rad(45), false)));
	// 	printf("Angle: %f | Result: %f\n", deg(rad), deg(unit_test_point(projected, rad(45), false)));
	// }

	// std::vector<float> p; p.reserve(1000);
	// for (int i = 1; i <= 500; i++) {
	// 	if (i % 40 == 0) dir *= -1;
	// 	robot::velo(dir, dir);

	// 	p.push_back(
	// 		(robot::left_motors.get_actual_velocity()
	// 		+robot::right_motors.get_actual_velocity())
	// 		/ 2.0f
	// 	);

	// 	pros::delay(20);
	// }
	// robot::brake();

	// print_vector(p, "L");

	while (true && !config::SIM_MODE) {
		for (auto& func : controls::ticks) {
			func();
		}
		pros::delay(10);
	}
}