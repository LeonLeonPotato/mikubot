#include "main.h"
#include "essential.h"
#include "autonomous/odometry.h"
#include "autonomous/pathing.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/solvers.h"
#include "autonomous/strategies.h"
#include "gui/opcontrolinfo.h"
#include "opcontrol/opcontrol.h"
#include "telemetry.h"

#include "api.h"

void initialize(void) {
	std::cout << "Initialize started" << std::endl;

	robot::init();
	odometry::start_task();
	// telemetry::start_task();
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
	autonrunner::init();

	strategies::functions.at(strategies::chosen_strategy)();
}

static void thug(std::string name, std::vector<float>& X, std::vector<float>& Y) {
	std::stringstream sts;
	sts << name << " = [";

	for (int i = 0; i < X.size(); i++) {
		float x = ((int) roundf(X[i] * 1000)) / 1000.0f;
		float y = ((int) roundf(Y[i] * 1000)) / 1000.0f;
		sts << "(" << x << ", " << y << ")";

		if (i != X.size() - 1) {
			sts << ", ";
		}
	}
	sts << "]";

	std::cout << sts.str() << std::endl;
}

void opcontrol(void) {
	// competition_initialize();

	Eigen::Vector2f goal = {1, 1};
	Eigen::Vector2f crosstrack = goal - robot::pos;
    Eigen::Matrix2f rotator;
	rotator << cosf(0.1), -sinf(0.1),
		sinf(0.1), cosf(0.1);
	Eigen::Vector2f crosstrack_local = rotator * crosstrack;
	printf("Rotated: (%f, %f)\n", crosstrack_local(0), crosstrack_local(1));

	// autonomous();
	// pros::delay(100);

	pathing::QuinticSpline test;
	test.points.emplace_back(0, 0);
	test.points.emplace_back(0, 68);
	test.solve_coeffs(pathing::BaseParams {
		.start_heading = 0,
		.start_magnitude = 0,
		.end_heading = 0,
		.end_magnitude = 0
	});
	std::cout << test.debug_out() << std::endl;

	double start = pros::micros() / 1e6f;
	test.profile_path({
		.start_v = 0,
		.end_v = 0,
		.max_speed = 5,
		.accel = 10,
		.decel = 10,
		.track_width = 40,
		.ds = 0.1,
		.resolution = 5000
	});
	printf("Sigma boy 1: %f\n", test.arc_parameter(0));
	printf("Sigma boy 2: %f\n", test.time_parameter(-1));
	printf("Profile path took %f seconds\n", pros::micros() / 1e6f - start);

	// std::vector<float> X; X.reserve(1000);
	// std::vector<float> Y_left; Y_left.reserve(1000);
	// std::vector<float> Y_center; Y_center.reserve(1000);
	// std::vector<float> Y_right; Y_right.reserve(1000);

	// for (auto& point : test.get_profile()) {
	// 	X.push_back(point.s);
	// 	Y_left.push_back(point.left_v);
	// 	Y_center.push_back(point.center_v);
	// 	Y_right.push_back(point.right_v);
	// }

	// thug("Y_left", X, Y_left);
	// printf("\n");
	// thug("Y_center", X, Y_center);
	// printf("\n");
	// thug("Y_right", X, Y_right);

	// pros::Motor motor(1, pros::MotorGears::blue);

	// std::vector<float> rots; rots.reserve(1000);
	// std::vector<float> currents; currents.reserve(1000);
	// std::vector<float> times; times.reserve(1000);

	// long long start = pros::micros();

	// while (true) {
	// 	double t = (pros::micros() - start) / 1e6f;
	// 	motor.move_voltage((int) (std::signbit(cosf(t * M_TWOPI)) * 2 - 1) * 12000);

	// 	rots.push_back(motor.get_actual_velocity() * M_TWOPI / 60.0f);
	// 	currents.push_back(motor.get_current_draw());
	// 	times.push_back(t);

	// 	if (times.size() > 300) break;
	// 	pros::delay(15);
	// }

	// motor.move_voltage(0);

	// std::stringstream stz;
	// stz << "I = [";
	// for (int i = 0; i < rots.size(); i++) {
	// 	float x = ((int) roundf(times[i] * 1000)) / 1000.0f;
	// 	float y = ((int) roundf(currents[i] * 1000)) / 1000.0f;
	// 	stz << "(" << x << ", " << y << ")";

	// 	if (i != rots.size() - 1) {
	// 		stz << ", ";
	// 	}
	// }
	// stz << "]";

	// std::cout << stz.str() << std::endl;
	
	std::cout << "Opcontrol started" << std::endl;

	for (auto& task : controls::start_tasks) {
		task();
	}
}