#include "main.h"
#include "essential.h"
#include "ansicodes.h"
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

static const auto PREFIX = ANSI_BOLD + ANSI_CYAN + "[Miku" + ANSI_GREEN + "bot] " + ANSI_RESET;

void initialize(void) {
	std::cout << PREFIX << "Initializing robot\n";

	robot::init();
	odometry::start_task();
	opcontrolinfo::init();

	if (!pros::competition::is_connected()) {
		std::cout << PREFIX << "Robot is not connected to the field controller, manually calling functions\n";
		competition_initialize();
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
	} while (autonselector::finished_selection == true);
	autonselector::destroy();
	std::cout << PREFIX << "Auton selection has finished\n";

	autonrunner::init();
}

void autonomous(void) {
	std::cout << PREFIX << "Running autonomous\n";
	autonrunner::init();

	strategies::functions.at(strategies::chosen_strategy)();
}

static void profiling_test(void) {
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
	std::cout << qs.debug_out() << std::endl;
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
	auto t = pros::micros() - start;
	pros::delay(20);
	printf("Profile path took %lld us\n", t);

	// std::cout << "[";
	// int cnt = 0;
	// for (auto& p : qs.get_profile()) {
	// 	std::cout << "(" + std::to_string(p.s) + ", " + std::to_string(p.left_v) << ")";
	// 	pros::delay(20);
	// 	cnt++;
	// 	if (cnt != qs.get_profile().size()) {
	// 		std::cout << ", ";
	// 	}
	// 	if (cnt % 10 == 0) {
	// 		std::cout << std::endl;
	// 	}
	// }
	// std::cout << "]" << std::endl;
}

static void is_it_actually_faster(void) {
	// asm volatile("cpsid i\n\tdsb\n\tisb");

	pathing::Polynomial2D<6> p = pathing::Polynomial2D<6>();
	p.x_poly.coeffs << 1, -49.2, 5.2, 2, 3, 4;
	p.y_poly.coeffs << 9.2, 8.888, 2.2, -4, 3, 4;

	Eigen::VectorXf times = Eigen::VectorXf::LinSpaced(10000, 0, 1);
	Eigen::Matrix2Xf res; res.resize(2, times.size());
	// asm volatile("" :: "r,m" (res));

	long long start = pros::micros();
	p.compute(times, res, 0);
	auto t1 = pros::micros() - start;
	printf("Polynomial2D vectorized took %lld us\n", t1);

	Eigen::VectorXf times2 = Eigen::VectorXf::LinSpaced(10000, 0, 1);
	Eigen::Matrix2Xf res2; res2.resize(2, times2.size());
	// asm volatile("" :: "r,m" (res2));
	
	start = pros::micros();
	for (int i = 0; i < 10000; i++) {
		res.col(i) = p.compute(times2.coeffRef(i), 0);
	}
	auto t2 = pros::micros() - start;
	printf("Polynomial2D single took %lld us\n", t2);

	// asm volatile("cpsie i\n\tdsb\n\tisb");
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	autonrunner::destroy(); pros::delay(10);
	autonselector::destroy(); pros::delay(10);
	opcontrolfun::init();

	profiling_test();
	// is_it_actually_faster();

	while (true) {
		for (auto& func : controls::ticks) {
			func();
		}
		pros::delay(10);
	}
}