#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"
#include "config.h"

#include "api.h"

using namespace strategies;

static const controllers::PIDArgs linear_args {
    .kp = 0.8,
    .ki = 0,
    .kd = -0.01,
    .integral_limit = 99999999.0f,
    .disable_integral_limit = 99999999.0f,
    .sign_switch_reset = false
};

static const controllers::PIDArgs angular_args {
    .kp = 0.5,
    .ki = 0,
    .kd = 0,
    .integral_limit = 99999999.0f,
    .disable_integral_limit = 99999999.0f,
    .sign_switch_reset = false
};

static const controllers::PIDArgs in_place_args {
    .kp = 1,
    .ki = 0,
    .kd = -0.025,
    .integral_limit = 99999999.0f,
    .disable_integral_limit = 99999999.0f,
    .sign_switch_reset = false
};

static void t1() {
    // robot::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // movement::PurePursuit pure_pursuit(30); // Pure pursuit controller with radius 100 cm
    // pure_pursuit.solver_override = solvers::Solver::Secant;
    // controllers::PID linear(linear_args);
    // controllers::PID angular(angular_args);

    // pathing::QuinticSpline path; // Empty quintic spline
    // path.points.emplace_back(0, 0);
    // path.points.emplace_back(0, 100);
    // path.points.emplace_back(70, 100);
    // path.set_relative(robot::pos); // Add all points with robot::pos

    // // We do not need to solve the coefficients because pathing will for us (:
    // Future<movement::MovementResult> fut = pure_pursuit.follow_path_async(
    //     path, 
    //     movement::PurePursuitParams {{ 
    //         .final_threshold = 10.0,
    //         .max_base_speed = 0.7,
    //         .force_recomputation = movement::RecomputationLevel::NONE,
    //         .timeout = 6000,
    //         .delay = 20,
    //     }}, 
    //     movement::PIDGroup {
    //         .angular = angular,
    //         .linear = linear
    //     }
    // );

    // // Demonstration of async ability
    // while (!fut.available()) {
    //     printf("Pos: (%f, %f) Theta diff: %f, dist: %f\n", robot::pos.x(), robot::pos.y(), 
    //         robot::angular_diff(path.points.back()), robot::distance(path.points.back()));
    //     pros::delay(20);
    // }

    // auto result = std::move(fut.get());
    // printf("Result: %d\n", result.code);
    // printf("Time: %d\n", result.time_taken_ms);
    // printf("Path recomputations: %d\n", result.num_path_recomputations);
    // printf("Time recomputations: %d\n", result.num_time_recomputations);
    // printf("Numerical error: %f\n", result.error);
    // printf("Spline parameter: %f\n", result.t);

    // robot::set_brake_mode(config::default_brake_mode);
}

static void t2() {
    
    // controllers::PID linear(linear_args);
    // controllers::PID angular(in_place_args);
    // movement::PIDGroup group {
    //     .angular = angular,
    //     .linear = linear
    // };

    // robot::intake.move_velocity(200);
    // robot::conveyor.move_velocity(200);

    // movement::simple::swing_to({0, 50}, group, false, 10000, 1.0, 10);
    // linear.reset(); angular.reset();

    // movement::simple::swing_to({-60, 150}, group, false, 10000, 1.0, 10);

    // linear.reset(); angular.reset();
    // robot::brake();
    // pros::delay(2000);
    // robot::intake.move_velocity(0);
    // robot::conveyor.move_velocity(0);

    // robot::brake();
}

void test_strategy::run(void) {
    pathing::QuinticSpline path; // Empty quintic spline
    path.points.emplace_back(0, 0);
    path.points.emplace_back(0, 100);
    path.points.emplace_back(70, 100);
    path.set_relative(robot::pos);
    path.solve_coeffs({
        .start_heading = robot::theta,
        .start_magnitude = 10,
        .end_heading = robot::theta,
        .end_magnitude = 0
    });
    path.profile_path({
        .start_v = 10 * robot::DRIVETRAIN_LINEAR_MULT,
        .end_v = 0,
        .max_speed = 55 * robot::DRIVETRAIN_LINEAR_MULT,
        .accel = 120 * robot::DRIVETRAIN_LINEAR_MULT,
        .decel = 95 * robot::DRIVETRAIN_LINEAR_MULT,
        .track_width = 39,
        .ds = 0.1,
        .resolution = 10000
    });

    movement::ramsete::RamseteParams params {{
        .timeout=10000
    }, {
        .beta=1.0,
        .zeta=0.7
    }};

    movement::ramsete::follow_path(path, params);

    robot::brake();
}

