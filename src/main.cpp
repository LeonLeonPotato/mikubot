#include "main.h"
#include "essential.h"
#include "opcontrol/driving.h"
#include "autonomous/odometry.h"
#include "autonomous/spline.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"
#include "gui/visiontest.h"
#include "autonomous/controllers.h"
#include "gui/utils.h"

#include "api.h"

#include "autonomous/pathing.h"
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
	competition_initialize();
	autonrunner::init();
	std::cout << "Opcontrol started" << std::endl;
	// driving::run();
	visiontest::init();
	controllers::PID pid(0.05, 0.005, 0.0, -999, 999, 999, -999);
	while (true) {
		if (robot::vision.get_object_count() > 0) {
			pros::vision_object_s_t biggest = robot::vision.get_by_sig(0, robot::signatures::test_id);
			int diff = biggest.x_middle_coord - 158;
			pid.register_error((float)diff);
			int power = (int) (pid.get());
			// if (fabs(power) < 3) continue;
			power = fmin(fmax(power, -10), 10);
			robot::motor.move(power);
		} else {
			pid.reset();
			robot::motor.brake();
		}
		pros::delay(10);
	}	
}