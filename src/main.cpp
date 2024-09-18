#include "main.h"
#include "api.h"

#include "robot.h"
#include "autonomous/spline.h"

#include <iostream>

void initialize(void) {
	// spline::init();
}

void disabled(void) {}

void competition_initialize(void) {}

void autonomous(void) {}

void opcontrol(void) {
	// std::cout << "Balls" << std::endl;
	// spline::QuinticSpline sp;
	// sp.points.emplace_back(0, 0);
	// sp.points.emplace_back(1, 0);
	// sp.points.emplace_back(1, 1);
	// sp.points.emplace_back(0, 1);
	// std::cout << std::to_string(sp.points.size()) << std::endl;
	// sp.solve_coeffs(0, 0, 0, 0);
	// std::cout << sp.debug_out() << std::endl;

	for (int i = 0; i < 1000; i++) {
		std::cout << "Hello World " << i << std::endl;
		for (int j = 0; j < 100; j++) asm ("nop");
	}
}