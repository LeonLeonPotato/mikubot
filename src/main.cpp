#include "main.h"
#include "api.h"

#include "robot.h"
#include "autonomous/spline.h"

#include <iostream>

void initialize(void) {
	spline::init();
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
	spline::QuinticSpline sp;
	sp.points.emplace_back(0, 0);
	sp.points.emplace_back(0, 1);
	sp.points.emplace_back(1, 1);
	sp.points.emplace_back(1, 0);
	sp.solve_coeffs(0, 0, 0, 0);
	std::cout << sp.debug_out() << std::endl;
}

void disabled(void) {}

void competition_initialize(void) {}

void autonomous(void) {}

void opcontrol(void) {

}