#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

void test_strategy::run(void) {
    robot::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    movement::PurePursuit pure_pursuit(100);

    auto path = pathing::QuinticSpline();
    path.points.emplace_back(robot::x, robot::y);
    path.points.emplace_back(robot::x - 53, robot::y + 136);
    path.points.emplace_back(robot::x - 15, robot::y + 269);
    path.points.emplace_back(robot::x + 100.6, robot::y + 308.5);
    path.points.emplace_back(robot::x + 191, robot::y + 263);
    path.points.emplace_back(robot::x + 195, robot::y + 106);
    path.points.emplace_back(robot::x + 96, robot::y + 35.5);

    Future<movement::MovementResult> fut = pure_pursuit.follow_path_async(path, movement::BaseMovementParams { 
        .always_recompute = true,
        .timeout = 1000,
        .delay = 5
    });

    while (!fut.available()) {
        // printf("Doing other stuff while pathing!\n");
        pros::delay(500);
    }

    auto result = std::move(fut.get());
    printf("Result: %d\n", result.code);
    printf("Time: %d\n", result.time_taken_ms);
    printf("Recomputations: %d\n", result.num_recomputations);
    printf("Error: %f\n", result.error);
    printf("T: %f\n", result.t);

    robot::set_brake_mode(robot::config::default_brake_mode);
}