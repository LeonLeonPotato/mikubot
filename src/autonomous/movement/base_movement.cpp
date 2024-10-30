#include "autonomous/movement.h"
#include "essential.h"

#include "api.h"

using namespace movement;

std::pair<float, float> BaseMovement::compute_initial_t_newton(void)
{
    Eigen::VectorXf guesses = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.05, maxt() - 1.05);
    return solvers::newton_vec(
        vec_func_, vec_deriv_, 
        guesses, 
        0, maxt(), 
        params.recomputation_iterations, params.recomputation_threshold
    );
}

std::pair<float, float> BaseMovement::compute_initial_t_secant(void) 
{
    Eigen::VectorXf t0 = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.05, maxt() - 1.1);
    Eigen::VectorXf t1 = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.10, maxt() - 1.05);

    return solvers::secant_vec(
        vec_func_,
        t0, t1, 
        0, maxt(), 
        params.recomputation_iterations, params.recomputation_threshold
    );
}

std::pair<float, float> BaseMovement::compute_updated_t_newton(float t) 
{
    return solvers::newton_single(
        func_, deriv_, 
        t, 
        t, maxt(), 
        params.update_iterations, params.update_threshold
    );
}

std::pair<float, float> BaseMovement::compute_updated_t_secant(float t) 
{
    return solvers::secant_single(
        func_, 
        t, fmin(maxt(), t + 1),
        t, maxt(), 
        params.update_iterations, params.update_threshold
    );
}

float BaseMovement::compute_updated_t_grad_desc(float t) 
{
    return solvers::gradient_descent_single(
        deriv_,
        t, 
        0, maxt(), 
        params.grad_desc_step_size, 
        params.update_iterations
    );
}

std::pair<float, float> BaseMovement::compute_initial_t(solvers::Solver solver)
{
    switch (solver) {
        case solvers::Solver::Newton:
            return compute_initial_t_newton();
        case solvers::Solver::Secant:
            return compute_initial_t_secant();
        default:
            return {-1, -1};
    }
}

std::pair<float, float> BaseMovement::compute_updated_t(solvers::Solver solver, float t)
{
    switch (solver) {
        case solvers::Solver::Newton:
            return compute_updated_t_newton(t);
        case solvers::Solver::Secant:
            return compute_updated_t_secant(t);
        default:
            return {-1, -1};
    }
}

void BaseMovement::recompute_path(int goal_i)
{
    printf("4");
    if (goal_i > 1) {
        for (int i = goal_i; i < path->points.size(); i++) {
            path->points[i - goal_i + 1] = path->points[i];
        }
        for (int i = 0; i < goal_i - 1; i++) {
            path->points.pop_back();
        }
    }
    printf("5");

    std::cout << "Recompute path pointer: " << (int) this->path << std::endl;

    path->points[0] = Eigen::Vector2f(robot::x, robot::y);
    printf("Set new initial point\n");

    pathing::BaseParams solve_params;
    solve_params_initializer(solve_params);

    path->solve_coeffs(solve_params);
}

// Do not question me on my generic pid parameters. Trust the femboy programmer~
void BaseMovement::init_generic_pid(controllers::PID& pid) {
    pid.kp = 800;
    pid.ki = 25;
    pid.kd = 400;
    pid.integral_limit = 1;
    pid.disable_integral_limit = 1;
    pid.sign_switch_reset = true;
}

void BaseMovement::init_generic_solve_params(pathing::BaseParams& solve_params) {
    solve_params.start_heading = robot::theta;
    solve_params.start_magnitude = 10 * robot::speed();
    solve_params.end_heading = 0;
    solve_params.end_magnitude = 0;
}
