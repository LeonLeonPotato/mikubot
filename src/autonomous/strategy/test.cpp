#include "autonomous/strategy/test.h"
#include "autonomous/controllers/pid.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/swing.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"
#include "subsystems.h"
#include "subsystems/impl/clamp.h"
#include "subsystems/impl/conveyor.h"

using namespace strategies;

static void part1_ring_side(void) {
    float mult = robot::match::team == 'B' ? 1 : -1;

    // pros::delay(2500);

    auto res1 = movement::simple::forward(
        25,
        {.reversed=true, .linear_exit_threshold = 1, .timeout = 3000},
        linear_pid
    );

    auto res2 = movement::simple::turn_towards(
        -pi/2 * mult,
        {.angular_exit_threshold = -1, .timeout = 800},
        in_place_pid
    );

    auto res3 = movement::simple::forward(
        10,
        {.reversed=true, .linear_exit_threshold = 0.3, .timeout = 500},
        linear_pid
    );

    robot::brake();
    auto& conveyor = subsystems::Conveyor::get_instance();
    conveyor.take_mutex();
    conveyor.set_desired_voltage(12000);
    pros::delay(700);

    auto res4 = movement::simple::turn_towards(
        pi,
        {.angular_exit_threshold = -1, .timeout = 600},
        in_place_pid
    );

    auto res5 = movement::simple::boomerang(
        {-60 * mult, 30, -pi/2 * mult}, 
        0.7f, 
        {.reversed=true, .linear_exit_threshold=4.0, .angular_exit_threshold=999, .timeout=3000}, 
        boomerang_group);

    auto res51 = movement::simple::turn_towards(
        -pi/2 * mult,
        {.reversed=true, .angular_exit_threshold = -1, .timeout = 600},
        in_place_pid
    );

    auto res52 = movement::simple::forward(
        20, 
        {.reversed=true, .linear_exit_threshold = 1.0, .max_linear_speed = 0.5f},
        linear_pid);

    robot::clamp.extend();
    pros::delay(100);
    conveyor.set_desired_voltage(-12000);
    pros::delay(200);
    conveyor.set_desired_voltage(12000);

    auto res6 = movement::simple::turn_towards(
        0,
        {.angular_exit_threshold = -1, .timeout = 800},
        in_place_pid
    );

    auto res7 = movement::simple::boomerang(
        {-130 * mult, 80, 0},
        0.3f,
        {.linear_exit_threshold=5.0, .angular_exit_threshold=999, .timeout=3000},
        swing_group
    );

    auto res72 = movement::simple::forward(
        5,
        {.linear_exit_threshold=0.3, .timeout=3000},
        linear_pid
    );

    pros::delay(800);
    conveyor.set_desired_voltage(-12000);
    pros::delay(200);
    conveyor.set_desired_voltage(12000);

    auto res9 = movement::simple::swing_to(
        {-80 * mult, 95},
        {.reversed=false, .linear_exit_threshold = 3.0, .timeout = 500},
        swing_group
    );

    robot::brake();
    pros::delay(1000);
}

static void part2_ring_side(void) {
    float mult = robot::match::team == 'B' ? 1 : -1;

    // auto res1 = movement::simple::turn_towards(
    //     {10 * mult, TILE * 2.1},
    //     {.angular_exit_threshold = -1, .timeout = 500},
    //     in_place_pid
    // );

    // auto res2 = movement::simple::boomerang(
    //     {6 * mult, 110, pi/4 * mult},
    //     0.5f,
    //     {.reversed=false, .linear_exit_threshold = 5.0, .angular_exit_threshold=999, .timeout = 3000},
    //     swing_group
    // );

    // robot::brake();
    // pros::delay(1000);

    // for (int i = 0; i < 2; i++) {
    //     auto backward = movement::simple::forward(
    //         TILE/2,
    //         {.reversed=true, .linear_exit_threshold = 0.3, .timeout = 500},
    //         linear_pid
    //     );

    //     controllers::PID ram_pid {
    //         {
    //             .kp = 0.2, .ki = 0.00, .kd = 0.000
    //         }
    //     };

    //     auto forward = movement::simple::forward(
    //         TILE/2,
    //         {.reversed=false, .linear_exit_threshold = 0.3, .timeout = 500},
    //         ram_pid
    //     );

    //     robot::brake();
    //     pros::delay(1000);
    // }

    // auto backward = movement::simple::forward(
    //     TILE/2,
    //     {.reversed=true, .linear_exit_threshold = 0.3, .timeout = 500},
    //     linear_pid
    // );

    // subsystems::Conveyor::get_instance().set_desired_voltage(0);

    auto ret = movement::simple::boomerang(  
        {-40 * mult, -1.2*TILE, pi},
        0.5f,
        {.linear_exit_threshold = 5.0, .angular_exit_threshold=999, .timeout = 3000},
        swing_group
    );

    auto ret2 = movement::simple::turn_towards(  
        {-40 * mult, -1*TILE},
        {.angular_exit_threshold=rad(30), .timeout = 3000},
        in_place_pid
    );

    robot::brake();

    // robot::brake();
    // pros::delay(1000);

    // auto backagain = movement::simple::forward(
    //     TILE,
    //     {.reversed=true, .linear_exit_threshold = 1.0, .timeout = 1500},
    //     linear_pid
    // );

    // auto res3 = movement::simple::turn_towards(
    //     -pi/2 * mult,
    //     {.angular_exit_threshold = -1, .timeout = 800},
    //     in_place_pid
    // );

    // auto res4 = movement::simple::forward(
    //     30,
    //     {.linear_exit_threshold = 0.3, .max_linear_speed=0.3f, .timeout = 5000},
    //     linear_pid
    // );

    robot::brake();
}

static void pos_goalrush_1(void) {
    float mult = robot::match::team == 'B' ? 1 : -1;

    robot::doinker.extend();

    movement::simple::forward(
        TILE * 2,
        {.reversed = true, .linear_exit_threshold = 2.0, .max_linear_speed = 0.5f}, 
        linear_pid
    );

    robot::clamp.extend();
    pros::delay(200);

    subsystems::Conveyor::get_instance().take_mutex();
    subsystems::Conveyor::get_instance().set_desired_voltage(12000);

    pros::delay(1000);

    // robot::clamp.retract();

    movement::simple::turn_towards(
        {-TILE * mult, -TILE * 2 + 25},
        {.angular_exit_threshold = -1, .timeout = 800},
        in_place_pid
    );

    movement::simple::forward(
        TILE * 1.5, 
        {.linear_exit_threshold = 1.5f, .timeout = 2000}, 
        linear_pid);
}

void test_strategy::run(void) {
    bool neg = (robot::match::team == 'R' && robot::match::side == 1)
        || (robot::match::team == 'B' && robot::match::side == -1);
    robot::chassis.take_drive_mutex();

    // std::vector<Pose> poses;
    // auto logger_task = get_logging_task(poses);

    if (neg) {
        part1_ring_side();
        part2_ring_side();
    } else {
        pos_goalrush_1();
    }

    // logger_task.remove();
    robot::chassis.give_drive_mutex();

    // print_poses(poses);
}

