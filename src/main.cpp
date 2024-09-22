#include "main.h"
#include "robot.h"
#include "autonomous/spline.h"
#include "gui/autonselector.h"
#include "gui/autonrunner.h"

#include "api.h"

#include <iostream>


void initialize(void) {
	std::cout << "Initialize started" << std::endl;

	spline::init();
	autonselector::init();
	// for (int size = 5; size < 100; size += 10) {
	// 	spline::QuinticSpline sp;
	// 	for (int i = 0; i < size; i++) {
	// 		float j = (float) i;
	// 		sp.points.emplace_back(j, j);
	// 	}

	// 	auto start = pros::millis();
	// 	sp.solve_coeffs(1, 0, -1, 0);
	// 	auto end = pros::millis();
	// 	auto duration = (end - start);

    // 	std::cout << "Quintic spline test took " << duration << " millis for size " << size << std::endl;	
	// }
}

void disabled(void) {
	std::cout << "Robot has been disabled" << std::endl;
}

void competition_initialize(void) {
	std::cout << "Comp init started" << std::endl;
}

void autonomous(void) {
	std::cout << "Auton started" << std::endl;
}

void opcontrol(void) {
	while (autonselector::finished_selection == false) {
		pros::delay(20);
	}

	std::cout << "Auton selector finished" << std::endl;

	autonselector::destroy();

	autonrunner::init();

	std::cout << "Opcontrol started" << std::endl;
}