#include "autonomous/movement.h"
#include "essential.h"

#include "api.h"

using namespace movement;

MovementResult BaseMovement::follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path) {
    controllers::PID pid;
    BaseMovement::init_generic_pid(pid);
    return follow_path_cancellable(cancel_ref, path, BaseMovementParams(), pid);
}

MovementResult BaseMovement::follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path, const BaseMovementParams& params) {
    controllers::PID pid;
    BaseMovement::init_generic_pid(pid);
    return follow_path_cancellable(cancel_ref, path, params, pid);
}

MovementResult BaseMovement::follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path, controllers::PID& pid) {
    return follow_path_cancellable(cancel_ref, path, BaseMovementParams(), pid);
}

std::pair<float, float> BaseMovement::compute_initial_t_newton(
    const pathing::BasePath& path, const BaseMovementParams& params,
    solvers::func_vec_t func, solvers::func_vec_t deriv) const
{
    Eigen::VectorXf guesses = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.05, path.maxt() - 1.05);
    return solvers::newton_vec(
        func, deriv, 
        guesses, 
        0, path.maxt(), 
        params.recomputation_iterations, params.recomputation_threshold
    );
}

std::pair<float, float> BaseMovement::compute_initial_t_secant(
    const pathing::BasePath& path, const BaseMovementParams& params,
    solvers::func_vec_t func) const
{
    Eigen::VectorXf t0 = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.05, path.maxt() - 1.1);
    Eigen::VectorXf t1 = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.10, path.maxt() - 1.05);

    return solvers::secant_vec(
        func,
        t0, t1, 
        0, path.maxt(), 
        params.recomputation_iterations, params.recomputation_threshold
    );
}

std::pair<float, float> BaseMovement::compute_updated_t_newton(
    const pathing::BasePath& path, const BaseMovementParams& params,
    solvers::func_t func, solvers::func_t deriv, float t) const
{
    return solvers::newton_single(
        func, deriv, 
        t, 
        t, path.maxt(), 
        params.update_iterations, params.update_threshold
    );
}

std::pair<float, float> BaseMovement::compute_updated_t_secant(
    const pathing::BasePath& path, const BaseMovementParams& params,
    solvers::func_t func, float t) const
{
    return solvers::secant_single(
        func, 
        t, fmin(path.points.size() - 1, t + 1),
        t, path.maxt(), 
        params.update_iterations, params.update_threshold
    );
}

float BaseMovement::compute_updated_t_grad_desc(
    const pathing::BasePath& path, const BaseMovementParams& params,
    solvers::func_t deriv, float t) const
{
    return solvers::gradient_descent_single(
        deriv,
        t, 
        0, path.maxt(), 
        params.grad_desc_step_size, 
        params.update_iterations
    );
}

std::pair<float, float> BaseMovement::compute_initial_t(
    const pathing::BasePath& path, const BaseMovementParams& params, 
    std::optional<solvers::func_vec_t> func,
    std::optional<solvers::func_vec_t> deriv,
    solvers::Solver solver) const
{
    if (solver == solvers::Solver::None)
        solver = get_solver(path);

    switch (solver) {
        case solvers::Solver::Newton:
            return compute_initial_t_newton(path, params, func.value(), deriv.value());
        case solvers::Solver::Secant:
            return compute_initial_t_secant(path, params, func.value());
        default:
            return {-1, -1};
    }
}

std::pair<float, float> BaseMovement::compute_updated_t(
    const pathing::BasePath& path, const BaseMovementParams& params, float t,
    std::optional<solvers::func_t> func,
    std::optional<solvers::func_t> deriv,
    solvers::Solver solver) const
{
    if (solver == solvers::Solver::None)
        solver = get_solver(path);

    switch (solver) {
        case solvers::Solver::Newton:
            return compute_updated_t_newton(path, params, func.value(), deriv.value(), t);
        case solvers::Solver::Secant:
            return compute_updated_t_secant(path, params, func.value(), t);
        default:
            return {-1, -1};
    }
}

void BaseMovement::recompute_path(pathing::BasePath& path, int goal_i) const
{
    if (goal_i > 1) {
        for (int i = goal_i; i < path.points.size(); i++) {
            path.points[i - goal_i + 1] = path.points[i];
        }
        for (int i = 0; i < goal_i - 1; i++) {
            path.points.pop_back();
        }
    }

    path.points[0] = Eigen::Vector2f(robot::x, robot::y);

    pathing::BaseParams solve_params;
    solve_params_initializer(solve_params);

    path.solve_coeffs(solve_params);
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
    solve_params.start_magnitude = fmax(10, robot::speed());
    solve_params.end_heading = 0;
    solve_params.end_magnitude = 0;
}
