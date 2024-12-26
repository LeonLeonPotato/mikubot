#include "autonomous/strategy/peak_strat.h"
#include "autonomous/strategy/utils.h"

using namespace strategies;

static void blue_right(void) {
    robot::clamp.retract();
    
    movement::simple::swing_to(
        {0, -95}, 
        {.reversed=true, .max_linear_speed=0.5f, .timeout=1000}, 
        swing_group);
    swing_group.reset();

    robot::brake();

    pros::delay(100);
    robot::clamp.extend();
    pros::delay(500);

    movement::simple::swing_to(
        {0, -75}, 
        {
            .reversed=false, 
            .max_linear_speed=0.3f, 
            .timeout=300
        }, 
        swing_group);
    swing_group.reset();

    robot::intake.move_voltage(12000);
    robot::conveyor.move_voltage(12000);
    pros::delay(750);
    robot::intake.move_voltage(-12000);
    robot::conveyor.move_voltage(-12000);

    int xto = -60;
    int yto = -95;

    if (robot::match::side == 1) xto = -xto;

    movement::simple::turn_towards(
        {xto, yto}, 
        {
            .exit_threshold=rad(5), 
            .timeout=750
        },
        in_place_pid);
    in_place_pid.reset();
    robot::brake();

    pros::delay(200);
    robot::intake.move_voltage(12000);
    robot::conveyor.move_voltage(12000);

    movement::simple::swing_to(
        {xto, yto}, 
        {
            .max_linear_speed=1.0f, 
            .timeout=1500
        }, 
        swing_group);
    swing_group.reset();

    pros::delay(100);
    robot::brake();
    
    // pros::delay(100);

    // movement::simple::swing_to(
    //     {-45, -135}, 
    //     {
    //         .exit_threshold=0.5f, 
    //         .max_linear_speed=0.5f,
    //         .timeout=1000
    //     },
    //     swing_group);
    // swing_group.reset();
    // robot::brake();

    pros::delay(3000);
}

void peak_strat::run(void) {
    blue_right();
}