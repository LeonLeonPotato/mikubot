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
    .kp = 1,
    .ki = 0,
    .kd = -0.025
};

static controllers::PID linear_pid(linear_args);
static controllers::PID angular_pid(angular_args);
static controllers::PID in_place_pid(in_place_args);

static const movement::PIDGroup path_group {
    .angular = angular_pid,
    .linear = linear_pid
};

static const movement::PIDGroup swing_group {
    .angular = in_place_pid,
    .linear = linear_pid
};

void peak_strat::run(void) {
    movement::simple::swing_to({0, -95}, swing_group, true, 1000, 0.6f);
    robot::brake();
    swing_group.reset();
    pros::delay(200);

    robot::clamp.extend();
    robot::intake.move_voltage(12000);
    robot::conveyor.move_voltage(12000);
    pros::delay(500);
    
    movement::simple::swing_to({-55, -95}, swing_group, false, 2000, 1.0f);
    
    robot::brake();
    pros::delay(100);

    movement::simple::turn_towards(-M_PI, in_place_pid, 2000, 0.01);
    in_place_pid.reset();
    
    pros::delay(100);
    movement::simple::swing_to({-45, -135}, swing_group, false, 1000, 0.5f);
    robot::brake();

    pros::delay(5000);
}

