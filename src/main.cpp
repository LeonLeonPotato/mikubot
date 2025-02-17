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

void initialize(void) {
	if (config::SIM_MODE) pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
	std::cout << PREFIX << "Initializing robot\n";

	robot::init();
	driverinfo::init();
	// telemetry::start_task();

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
			debugscreen::init();
			autonomous();
		}
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

#include "nlopt/nlopt.hpp"
#include "autodiff/reverse/var.hpp"

using var = autodiff::Variable<double>;

struct RobotParams {
	float track_width;
	float gain, tc;
	float dt;
} robot_params = {0.3, 0.517, 0.16, 0.05};
constexpr int N = 5;
float max_scale = std::pow(2, N);

// Define the objective function
var objective_function(const std::vector<var>& x) {
	var rx = 0.0, ry = 0.0, theta = 0.0, vl = 0.0, vr = 0.0;
	var cost = 0.0;
	for (int i = 0; i < 2*N; i += 2) {
		vl += (robot_params.gain * x[i] - vl) / robot_params.tc * robot_params.dt;
		vr += (robot_params.gain * x[i+1] - vr) / robot_params.tc * robot_params.dt;
		var v = (vl + vr) / 2.0;
		var w = (vl - vr) / robot_params.track_width;
		var dtheta = w * robot_params.dt, travel = v * robot_params.dt;
		var half = dtheta / 2.0;
		var s = sinc(half);
		rx += travel * s * sin(theta + half);
		ry += travel * s * cos(theta + half);
		theta += dtheta;
	
		cost += (pow(rx - 5, 2) + pow(ry - 10, 2)) * (1 + i) / N;
	}
	printf("Cost: %f\n", autodiff::val(cost));
	return cost;
}

void verify(const std::vector<var>& x) {
	var rx = 0.0, ry = 0.0, theta = 0.0, vl = 0.0, vr = 0.0;
	var cost = 0.0;
	for (int i = 0; i < N; i++) {
		vl += (robot_params.gain * x[2*i] - vl) / robot_params.tc * robot_params.dt;
		vr += (robot_params.gain * x[2*i+1] - vr) / robot_params.tc * robot_params.dt;
		var v = (vl + vr) / 2.0;
		var w = (vl - vr) / robot_params.track_width;
		var dtheta = w * robot_params.dt, travel = v * robot_params.dt;
		var half = dtheta / 2.0;
		var s = sinc(half);
		rx += travel * s * sin(theta + half);
		ry += travel * s * cos(theta + half);
		theta += dtheta;
		printf("Time step %d: pos = [%f, %f, %f]\n",
			i, autodiff::val(rx), autodiff::val(ry), autodiff::val(theta));
		

		cost += (pow(rx - 50, 2) + pow(ry - 50, 2)) * (1 + i) / (2*N);
	}
}

// Helper function that takes a vector and an index sequence
template <typename Func, typename Vector, std::size_t... Indices>
auto get_derivatives_with_indices(Func&& f, const Vector& x_ad, std::index_sequence<Indices...>) {
    return autodiff::derivatives(f, autodiff::wrt(x_ad[Indices]...));
}

// Wrapper function to make it easier to use
template <typename Func, typename Vector>
auto get_derivatives(Func&& f, const Vector& x_ad) {
    return get_derivatives_with_indices(
        std::forward<Func>(f), x_ad, std::make_index_sequence<2*N>{}
    );
}

// Wrap the autodiff function to match the nlopt interface
double nlopt_objective_function(unsigned int n, const double* x, double* grad, void* data) {
    // Convert the input x to autodiff variables
    std::vector<var> x_ad;
	for (int i = 0; i < n; i++) {
		x_ad.push_back((float) x[i]);
	}
	var f = objective_function(x_ad);

	auto grad_ad = get_derivatives(f, x_ad);

	// Copy the gradient to the output grad
	for (int i = 0; i < n; i++) {
		grad[i] = autodiff::val(grad_ad[i]);
	}

	// Return the value of the function

	return autodiff::val(f);
}

int asdasdasd() {
    // Create an optimization problem object
    nlopt::opt opt(nlopt::GD_MLSL, 2*N);  // LD_MMA is a chosen optimization algorithm, 1 means 1 variable

    // Set the objective function
    opt.set_min_objective(nlopt_objective_function, nullptr);

    // Set the lower and upper bounds for the variable
    std::vector<double> lb(2*N, -12);  // Lower bound for x is 0
    std::vector<double> ub(2*N, 12);  // Upper bound for x is 5
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
	opt.set_ftol_rel(0.01);

    // std::vector<double> x {
	// 	12, 4,
	// 	-2, 4,
	// 	6, 4
	// };

	std::vector<double> x(2*N, rand() % 12);
	printf("Guess X[0]: %f\n", x[0]);

    // Perform the optimization
	long long start = pros::millis();
    double minf;  // To hold the minimum value
    nlopt::result result = opt.optimize(x, minf);
	long long end = pros::millis();
	printf("Time taken: %lld\n", end - start);

    // Output the result
    std::cout << "Result: " << result << std::endl;
    std::cout << "Minimum value of the function: " << minf << std::endl;
	for (int i = 0; i < 2*N; i++) {
		std::cout << "x[" << i << "] = " << x[i] << std::endl;
	}

	// Test the result
	std::vector<var> x_ad;
	for (int i = 0; i < 2*N; i++) {
		x_ad.push_back(x[i]);
	}
	verify(x_ad);
	

    return 0;
}

void opcontrol(void) {
	std::cout << PREFIX << "Operator control started\n";
	if (!config::SIM_MODE) autonrunner::destroy();
	if (!config::SIM_MODE) autonselector::destroy();

	// test();

	// asdasdasd();

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