#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

void test_strategy::run(void) {
    robot::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Pure pursuit with radius 100 cm
    movement::PurePursuit pure_pursuit(100);

    auto path = pathing::QuinticSpline();
    path.points.emplace_back(0, 0);
    path.points.emplace_back(-53, 136);
    path.points.emplace_back(-15, 269);
    path.points.emplace_back(100.6, 308.5);
    path.points.emplace_back(191, 263);
    path.points.emplace_back(195, 106);
    path.points.emplace_back(96, 35.5);
    path.set_relative(robot::pos());

    Future<movement::MovementResult> fut = pure_pursuit.follow_path_async(path, movement::MovementParams { 
        .force_recomputation = movement::RecomputationLevel::NONE,
        .timeout = 100,
        .delay = 20
    });

    while (!fut.available()) {
        printf("Doing other stuff while pathing!\n");
        pros::delay(20);
    }

    auto result = std::move(fut.get());
    printf("Result: %d\n", result.code);
    printf("Time: %d\n", result.time_taken_ms);
    printf("Path recomputations: %d\n", result.num_path_recomputations);
    printf("Time recomputations: %d\n", result.num_time_recomputations);
    printf("Error: %f\n", result.error);
    printf("T: %f\n", result.t);

    robot::set_brake_mode(robot::config::default_brake_mode);
}