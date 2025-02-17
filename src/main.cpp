#include "main.h"
#include "autodiff/reverse/var/var.hpp"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "config.h"
#include "essential.h"
#include "ansicodes.h"
#include "gui/debugscreen.h"
#include "Eigen/Dense"
#include "mathtils.h"
#include "subsystems.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "telemetry.h" // IWYU pragma: keep

#include "autonomous/strategies.h"

#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/funnymiku.h"
#include "gui/driverinfo.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "pros/apix.h" // IWYU pragma: keep

static bool real;

void initialize(void) {
	if (config::SIM_MODE) pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
	std::cout << PREFIX << "Initializing robot\n";

	int connected = 0;
	for (int port = 1; port <= 21; port++) {
		if (pros::c::get_plugged_type(port) != pros::c::E_DEVICE_NONE) {
			connected++;
		}
	}
	real = connected > 10;
	if (real) {
		printf("%sRobot %sprobably is not%s a raw brain (connected to %d devices)\n", 
			CPREFIX, ANSI_CYAN.c_str(), ANSI_RESET.c_str(), connected);
		robot::init();
		driverinfo::init();

		for (auto& subsystem : subsystems::subsystems) {
			if (subsystem->has_api()) {
				subsystem->start_api_task();
			}
		}
	
		if (!pros::competition::is_connected()) {
			std::cout << PREFIX << "Robot is not connected to the field controller, manually calling functions\n";
			if (!config::SIM_MODE) {
				// competition_initialize();
				robot::match::team = 'R';
				// pros::delay(100);
				// debugscreen::init();
				// autonomous();
			}
		}
	} else {
		printf("%sRobot %sprobably is%s a raw brain (connected to %d devices)\n", 
			CPREFIX, ANSI_RED.c_str(), ANSI_RESET.c_str(), connected);
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
	pros::delay(50);

	autonselector::destroy();

	std::cout << PREFIX << "Auton selection has finished\n";
}

void autonomous(void) {
	std::cout << PREFIX << "Running autonomous\n";
	// autonrunner::init();

	strategies::functions.at(strategies::chosen_strategy)();

	// autonrunner::destroy();
}

static void collect_pid_data(void) {
	robot::chassis.take_drive_mutex();

	std::vector<std::pair<float, float>> points;

	robot::velo(-0.5, 0.5);

	long long start = pros::micros();
	while ((pros::micros() - start) / 1e6f < 5) {
		float pos = robot::theta();
		points.push_back({(pros::micros() - start) / 1e6f, pos});
		pros::delay(10);
	}

	robot::velo(0, 0);

	std::cout << PREFIX << "Data collection finished\n";

	std::cout << "X = \\left[";
	for (int i = 0; i < points.size(); i++) {
		auto point = points[i];
		std::cout << "\\left(" << point.first << ",\\ " << point.second << "\\right)";
		if (i != points.size() - 1) {
			std::cout << ", ";
		}

		if (i % 5 == 0) {
			std::cout << "\n";
			std::cout.flush();
			pros::delay(100);
		}
	}

	std::cout << "\\right]\n";

	robot::chassis.give_drive_mutex();
}

static void collect_drivetrain_data(void) {
	robot::chassis.take_drive_mutex();
	
	for (float volt = 1; volt <= 12; volt += 1.5) {
		printf("%sSetting voltage to %f\n", CPREFIX, volt);
		printf("%sPress enter to continue\n", CPREFIX);
		std::cin.get();
		robot::chassis.set_voltage(volt / 12, volt / 12);
		
		std::vector<std::pair<float, float>> data;

		long long start = pros::micros();
		for (int i = 0; i < 100; i++) {
			data.push_back({(pros::micros() - start) / 1e6f, robot::right_motors.get_raw_velocity_average()});
			pros::delay(10);
		}
		robot::chassis.brake();

		printf("%sData collection finished\n", CPREFIX);

		printf("X = \\left[");
		for (int i = 0; i < data.size(); i++) {
			auto point = data[i];
			printf("\\left(%f,\\ %f\\right)", point.first, point.second);
			if (i != data.size() - 1) {
				printf(",");
			}

			if (i % 5 == 0) {
				printf("\n");
				fflush(stdout);
				pros::delay(100);
			}
		}
		printf("\\right]\n");
	}
}

static void collect_odom_centering_data(void) {
	robot::chassis.take_drive_mutex();

	printf("%sStarting data collection\n", CPREFIX);
	printf("%sPress enter to continue\n", CPREFIX);
	std::cin.get();


	std::vector<std::pair<float, float>> data;
	robot::chassis.set_velocity(0.2, -0.2);
	
	long long start = pros::micros();
	for (int i = 0; i < 500; i++) {
		data.push_back({
			robot::x(),
			robot::y()
		});
		pros::delay(10);
	}
	robot::chassis.brake();

	printf("%sData collection finished\n", CPREFIX);

	printf("X = \\left[");
	for (int i = 0; i < data.size(); i++) {
		auto point = data[i];
		printf("\\left(%f,\\ %f\\right)", point.first, point.second);
		if (i != data.size() - 1) {
			printf(",");
		}

		if (i % 5 == 0) {
			printf("\n");
			fflush(stdout);
			pros::delay(100);
		}
	}
	printf("\\right]\n");
}

static void test(void) {
	pathing::CubicSpline cubic {
		{{0, 0}, {50, 50}, {10, 0}}
	};

	cubic.solve_coeffs(pathing::CubicSpline::natural_conditions, pathing::CubicSpline::natural_conditions);
	std::cout << cubic.debug_out() << std::endl;

	cubic.profile_path({
		.start_v = 0.001, .end_v = 0.001,
		.max_speed = 10,
		.accel = 1,
		.decel = 0.5,
		.track_width = robot::DRIVETRAIN_WIDTH,
		.friction_coeff = 0.5,
		.ds = 0.1
	});

	auto& profile = cubic.get_profile();

	printf("X = \\left[");
	for (int i = 0; i < profile.size(); i++) {
		auto point = profile[i];
		printf("\\left(%f,\\ %f\\right)", point.real_time, point.center_v);
		if (i != profile.size() - 1) {
			printf(",");
		}

		if (i % 5 == 0) {
			printf("\n");
			fflush(stdout);
			pros::delay(100);
		}
	}
	printf("\\right]\n");
}

static void test_motors(void) {
	hardware::Motor test_motor (
		2, hardware::Gearset::GREEN, hardware::BrakeMode::BRAKE,
		{
			.kv = 48.933, .ka = 2.300, .kf = 143.6,
			.pid_args = {.kp = 0, .ki = 0, .kd = 0}
		},
		0.0f);
	test_motor.acquire_mutex();

	std::vector<std::pair<float, float>> data;
	std::vector<float> times;

	auto start = pros::micros();
	while (true) {
		times.push_back((pros::micros() - start) / 1e6);
		test_motor.set_desired_velocity(sin(times.back() * 5) * 200, 1000 * cos(times.back() * 5));
		data.push_back({
			test_motor.get_raw_velocity_average(),
			test_motor.get_filtered_velocity()
		});
		pros::delay(10);

		if (times.size() > 100) break;
	}
	test_motor.brake();

	std::cout << "X = \\left[";
	for (int i = 0; i < times.size(); i++) {
		std::cout << "\\left(" << times[i] << ",\\ " << data[i].first << "\\right)";
		if (i != times.size() - 1) {
			std::cout << ", ";
		}

		if (i % 5 == 0) {
			std::cout << "\n";
			std::cout.flush();
			pros::delay(100);
		}
	}
	std::cout << "\\right]\n";

	std::cout << "Y = \\left[";
	for (int i = 0; i < times.size(); i++) {
		std::cout << "\\left(" << times[i] << ",\\ " << data[i].second << "\\right)";
		if (i != times.size() - 1) {
			std::cout << ", ";
		}

		if (i % 5 == 0) {
			std::cout << "\n";
			std::cout.flush();
			pros::delay(100);
		}
	}
	std::cout << "\\right]\n";
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	if (!config::SIM_MODE && real) autonrunner::destroy();
	if (!config::SIM_MODE && real) autonselector::destroy();

	test_motors();

	// test();

	// test_motor_groups();

	// robot::chassis.take_drive_mutex();
	// pros::delay(1000);
	// robot::chassis.give_drive_mutex();

	// collect_odom_centering_data();

	for (auto& subsystem : subsystems::subsystems) {
		subsystem->take_mutex();
	}

	while (true && !config::SIM_MODE) {
		for (auto& subsystem : subsystems::subsystems) {
			subsystem->tick();
		}
		pros::delay(10);
	}
}