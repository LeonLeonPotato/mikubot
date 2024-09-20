#include "main.h"
#include "api.h"
#include "pros/apix.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/lv_conf.h"

#include "robot.h"
#include "autonomous/spline.h"

#include <iostream>

lv_obj_t *label;

void lv_example_get_started_1(void)
{
    /*Create a white label, set its text and align it to the center*/
	std::cout << lv_scr_act() << std::endl;
    label = lv_label_create(lv_scr_act());
	std::cout << "1" << std::endl;
    lv_label_set_text(label, "Hello world");
	std::cout << "2" << std::endl;
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
	std::cout << "3" << std::endl;
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
	std::cout << "4" << std::endl;
}


void initialize(void) {
	std::cout << "Initialize started" << std::endl;
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
	pros::delay(50);
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
	lv_example_get_started_1();
}