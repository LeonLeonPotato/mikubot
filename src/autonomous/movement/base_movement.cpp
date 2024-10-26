#include "autonomous/movement.h"
#include "essential.h"

#include "api.h"

using namespace movement;

std::pair<float, float> utils::compute_initial_t_newton(BaseMovement& mover)
{
    Eigen::VectorXf guesses = Eigen::VectorXf::LinSpaced(32, 0.05, mover.maxt() - 1.05);
    return solvers::newton_vec(
        mover.vec_func(), mover.vec_deriv(), 
        guesses, 
        0, mover.maxt(), 
        mover.recomputation_iterations,
        mover.recomputation_threshold
    );
}

std::pair<float, float> utils::compute_initial_t_secant(BaseMovement& mover) 
{
    Eigen::VectorXf t0 = Eigen::VectorXf::LinSpaced(32, 0.05, mover.maxt() - 1.1);
    Eigen::VectorXf t1 = Eigen::VectorXf::LinSpaced(32, 0.10, mover.maxt() - 1.05);

    return solvers::secant_vec(
        mover.vec_func(),
        t0, t1, 
        0, mover.maxt(), 
        mover.recomputation_iterations,
        mover.recomputation_threshold
    );
}

std::pair<float, float> utils::compute_updated_t_newton(BaseMovement& mover, float t) 
{
    return solvers::newton_single(
        mover.func(), mover.deriv(), 
        t, 
        t, mover.maxt(), 
        mover.update_iterations,
        mover.update_threshold
    );
}

std::pair<float, float> utils::compute_updated_t_secant(BaseMovement& mover, float t) 
{
    return solvers::secant_single(
        mover.func(), 
        t, t + 1,
        t, mover.maxt(), 
        mover.update_iterations,
        mover.update_threshold
    );
}

float utils::compute_updated_t_grad_desc(BaseMovement& mover, float t) 
{
    return solvers::gradient_descent_single(
        mover.deriv(), 
        t, 
        0, mover.maxt(), 
        mover.grad_desc_step_size, 
        mover.update_iterations
    );
}

std::pair<float, float> utils::compute_initial_t(solvers::Solver solver, BaseMovement& mover)
{
    switch (solver) {
        case solvers::Solver::Newton:
            return compute_initial_t_newton(mover);
        case solvers::Solver::Secant:
            return compute_initial_t_secant(mover);
        default:
            return {-1, -1};
    }
}

std::pair<float, float> utils::compute_updated_t(solvers::Solver solver, BaseMovement& mover, float t)
{
    switch (solver) {
        case solvers::Solver::Newton:
            return compute_updated_t_newton(mover, t);
        case solvers::Solver::Secant:
            return compute_updated_t_secant(mover, t);
        default:
            return {-1, -1};
    }
}

void utils::recompute_path(BaseMovement& mover, int goal_i)
{
    if (goal_i > 1) {
        for (int i = goal_i; i < mover.path.points.size(); i++) {
            mover.path.points[i - goal_i + 1] = mover.path.points[i];
        }
        for (int i = 0; i < goal_i - 1; i++) {
            mover.path.points.pop_back();
        }
    }

    mover.path.points[0] = Eigen::Vector2f(robot::x, robot::y);
    mover.solve_params.start_heading = robot::theta;
    mover.solve_params.start_magnitude = 10;
    mover.path.solve_coeffs(mover.solve_params);
}

// Do not question me on my generic pid parameters. Trust the femboy programmer~
void BaseMovement::init_generic_pid(controllers::PID& pid) {
    pid.kp = 800;
    pid.ki = 0;
    pid.kd = 400;
    pid.integral_min = -infinity();
    pid.integral_max = infinity();
    pid.disable_integral_lower = -infinity();
    pid.disable_integral_upper = 1;
}

void movement::goto_pos_tick(const Eigen::Vector2f& point, float distance_coeff, controllers::PID& pid) {
    float theta_diff = robot::angular_diff(point);
    float dist = robot::distance(point);
    float dist_vel = fmin(dist * distance_coeff, 127);
    pid.register_error(fabs(theta_diff));
    float turn_vel = pid.get();
    robot::volt(
        dist_vel + theta_diff * turn_vel, 
        dist_vel - theta_diff * turn_vel
    );
}

void movement::turn_towards_tick(float angle, controllers::PID& pid) {
    float theta_diff = robot::angular_diff(angle);
    pid.register_error(fabs(theta_diff));
    float turn_vel = pid.get();
    robot::volt(
        theta_diff * turn_vel, 
        -theta_diff * turn_vel
    );
}

void movement::goto_pos(const Eigen::Vector2f& point, float distance_coeff, float threshold) {
    controllers::PID pid; BaseMovement::init_generic_pid(pid);
    
    while (robot::distance(point) > threshold) {
        goto_pos_tick(point, distance_coeff, pid);
        pros::delay(20);
    }
}

void movement::turn_towards(float angle, float threshold) {
    controllers::PID pid; BaseMovement::init_generic_pid(pid);

    while (fabs(robot::angular_diff(angle)) > threshold) {
        turn_towards_tick(angle, pid);
        pros::delay(20);
    }
}

void movement::goto_pose(const Eigen::Vector2f& point, float angle, float distance_coeff, float threshold, float theta_threshold) {
    controllers::PID pid; BaseMovement::init_generic_pid(pid);

    while (robot::distance(point) > threshold) {
        goto_pos_tick(point, distance_coeff, pid);
        pros::delay(20);
    }

    pid.reset();

    while (fabs(robot::angular_diff(angle)) > theta_threshold) {
        turn_towards_tick(angle, pid);
        pros::delay(20);
    }

    robot::brake();
}