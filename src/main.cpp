#include "main.h"
#include "api.h"

#include "robot.h"
#include "autonomous/spline.h"

#include <chrono>
#include <iostream>

using namespace std::chrono;

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
}

void disabled(void) {}

void competition_initialize(void) {}

void autonomous(void) {}

void opcontrol(void) {

}