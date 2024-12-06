#include "autonomous/strategy/peak_strat.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"
#include "config.h"

#include "api.h"

#define rad(x) (x * M_PI / 180.0)

using namespace strategies;

static const controllers::PIDArgs linear_args {
    .kp = 1.0f / 30.0f,
    .ki = 0,
    .kd = 0
};

static const controllers::PIDArgs angular_args {
    .kp = 1.0,
    .ki = 0,
    .kd = -0.0
};

static const controllers::PIDArgs in_place_args {
    .kp = 0.8,
    .ki = 0,
    .kd = -0.1
};

static controllers::PID linear_pid(linear_args);
static controllers::PID angular_pid(angular_args);
static controllers::PID in_place_pid(in_place_args);

static const movement::PIDGroup path_group {
    .angular = angular_pid,
    .linear = linear_pid
};

static const movement::PIDGroup swing_group {
    .angular = angular_pid,
    .linear = linear_pid
};

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
        {.reversed=false, .max_linear_speed=0.3f, .timeout=300}, 
        swing_group);
    swing_group.reset();

    robot::intake.move_voltage(12000);
    robot::conveyor.move_voltage(12000);
    pros::delay(750);
    robot::intake.move_voltage(-12000);
    robot::conveyor.move_voltage(-12000);

    movement::simple::turn_towards(
        rad(-120), 
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
        {-55-5, -95-5}, 
        {
            .max_linear_speed=0.8f, 
            .timeout=1500
        }, 
        swing_group);
    swing_group.reset();
    
    robot::brake();
    pros::delay(100);

    movement::simple::turn_towards(
        -M_PI, 
        {
            .exit_threshold=rad(5), 
            .timeout=1000
        }, 
        in_place_pid);
    in_place_pid.reset();
    
    pros::delay(100);

    movement::simple::swing_to(
        {-45, -135}, 
        {
            .exit_threshold=0.5f, 
            .max_linear_speed=0.5f,
            .timeout=1000
        },
        swing_group);
    swing_group.reset();
    robot::brake();

    pros::delay(3000);
}

void peak_strat::run(void) {
    blue_right();
}