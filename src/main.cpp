#include "main.h"
#include "api.h"

#include "robot.h"
#include "autonomous/spline.h"

#include <iostream>

void on_center_button() {
std::cout << "Button started" << std::endl;
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

void initialize(void) {
	std::cout << "Initialize started" << std::endl;
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
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

void disabled(void) {
	std::cout << "Disabled started" << std::endl;
}

void competition_initialize(void) {
	std::cout << "Comp init started" << std::endl;
}

void autonomous(void) {
	std::cout << "Auton started" << std::endl;
}

void opcontrol(void) {
	std::cout << "Opcontrol started" << std::endl;
  while (true) {
    pros::lcd::print(0, "Buttons Bitmap: %d\n", pros::lcd::read_buttons());
    pros::delay(20);
  }
}