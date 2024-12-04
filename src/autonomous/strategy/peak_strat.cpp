#include "autonomous/strategy/peak_strat.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"
#include "config.h"

#include "api.h"

using namespace strategies;

static const controllers::PIDArgs linear_args {
    .kp = 1.0f / 30.0f,
    .ki = 0,
    .kd = 0
};

static const controllers::PIDArgs angular_args {
    .kp = 0.4,
    .ki = 0,
    .kd = 0
};

static const controllers::PIDArgs in_place_args {
    .kp = 2.0,
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

void peak_strat::run(void) {
    movement::simple::swing_to(
        {0, -95}, 
        {.reversed=true, .max_linear_speed=0.6f, .timeout=1000}, 
        swing_group);
        
    swing_group.reset();

    robot::brake();
    pros::delay(200);

    robot::clamp.extend();
    robot::intake.move_voltage(12000);
    robot::conveyor.move_voltage(12000);
    pros::delay(500);

    movement::simple::turn_towards(
        -M_PI_2, 
        {.exit_threshold=0.017, .timeout=2000}, 
        in_place_pid);

    in_place_pid.reset();
    pros::delay(1000);
    
    // movement::simple::swing_to(
    //     {-55, -95}, 
    //     {.max_linear_speed=1.0f, .timeout=2000}, 
    //     swing_group);
    
    // robot::brake();
    // pros::delay(100);

    // movement::simple::turn_towards(
    //     -M_PI, 
    //     {.exit_threshold=0.017, .timeout=2000}, 
    //     in_place_pid);

    // in_place_pid.reset();
    
    // pros::delay(100);
    // movement::simple::swing_to(
    //     {-45, -135}, 
    //     {.exit_threshold=0.5f, .timeout=1000},
    //     swing_group);

    robot::brake();

    pros::delay(5000);
}

