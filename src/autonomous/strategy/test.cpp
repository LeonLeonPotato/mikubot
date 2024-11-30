#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

void test_strategy::run(void) {
    robot::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    movement::PurePursuit pure_pursuit(100); // Pure pursuit controller with radius 100 cm
    controllers::PID linear {0.1, 0, 0.01}; // Example pids (I will tune tmrw)
    controllers::PID angular {0.1, 0, 0.01};

    pathing::QuinticSpline path; // Empty quintic spline
    path.points.emplace_back(0, 0);
    path.points.emplace_back(0, 100);
    path.points.emplace_back(100, 100); // Populate with points
    path.set_relative(robot::pos); // Add all points with robot::pos

    // We do not need to solve the coefficients because pathing will for us (:
    Future<movement::MovementResult> fut = pure_pursuit.follow_path_async(
        path, 
        movement::PurePursuitParams {{ 
            .force_recomputation = movement::RecomputationLevel::NONE,
            .timeout = 100,
            .delay = 20
        }}, 
        movement::PIDGroup {
            .angular = angular,
            .linear = linear
        }
    );

    // Demonstration of async ability
    while (!fut.available()) {
        printf("Doing other stuff while pathing!\n");
        pros::delay(20);
    }

    auto result = std::move(fut.get());
    printf("Result: %d\n", result.code);
    printf("Time: %d\n", result.time_taken_ms);
    printf("Path recomputations: %d\n", result.num_path_recomputations);
    printf("Time recomputations: %d\n", result.num_time_recomputations);
    printf("Numerical error: %f\n", result.error);
    printf("Spline parameter: %f\n", result.t);

    robot::set_brake_mode(robot::config::default_brake_mode);
}