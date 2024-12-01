#include "autonomous/movement.h"
#include "essential.h"

#include "api.h"

using namespace movement;

MovementResult BaseMovement::follow_path_cancellable(volatile bool& cancel_ref, pathing::BasePath& path, PIDGroup pids) const {
    return follow_path_cancellable(cancel_ref, path, get_global_params(), pids);
}

std::pair<float, float> BaseMovement::compute_initial_t(
    const pathing::BasePath& path, const MovementParams& params,
    const solvers::FunctionGroup& funcs, 
    solvers::Solver solver) const 
{
    if (solver == solvers::Solver::None) {
        solver = get_solver(path);
    }

    switch (solver) {
        case solvers::Solver::Newton: {
            Eigen::VectorXf guess = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.01, path.maxt() - 0.01);

            return solvers::newton_vec(funcs, guess, 0, path.maxt(), params.recomputation_iterations, params.recomputation_threshold);
            break;
        }
        case solvers::Solver::Secant: {
            Eigen::VectorXf t0 = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.05, path.maxt() - 0.10);
            Eigen::VectorXf t1 = Eigen::VectorXf::LinSpaced(params.recomputation_guesses, 0.10, path.maxt() - 0.05);

            return solvers::secant_vec(funcs, t0, t1, 0, path.maxt(), params.recomputation_iterations, params.recomputation_threshold);
            break;
        }
        default: {
            throw std::runtime_error("Invalid / Unsupported solver: " + std::to_string((int) solver));
        }
    }

    __builtin_unreachable();
}

std::pair<float, float> BaseMovement::compute_updated_t(
    pathing::BasePath& path, const MovementParams& params,
    const solvers::FunctionGroup& funcs, float t, 
    solvers::Solver solver) const 
{
    if (solver == solvers::Solver::None) {
        solver = get_solver(path);
    }

    switch (solver) {
        case solvers::Solver::Newton: {
            return solvers::newton_single(funcs, t, 0, path.maxt(), params.update_iterations, params.update_threshold);
        }
        case solvers::Solver::Secant: {
            const float t1 = fmin(t + 1, path.maxt());
            const float t0 = t - ((int) (t1 == t)) * 0.1;
            return solvers::secant_single(funcs, t0, t1, 0, path.maxt(), params.update_iterations, params.update_threshold);
        }
        case solvers::Solver::GradientDescent: {
            return solvers::gradient_descent_single(funcs, t, 0, path.maxt(), params.grad_desc_step_size, params.update_iterations);
        }
        default: {
            throw std::runtime_error("Invalid / Unsupported solver: " + std::to_string((int) solver));
        }
    }

    __builtin_unreachable();
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

    path.points[0] = robot::pos;

    path_solver(path);
    printf("Recomputed path\n");
}

void BaseMovement::solve_path_default(pathing::BasePath& path) {
    pathing::BaseParams solve_params;

    solve_params.start_heading = robot::theta;
    solve_params.start_magnitude = robot::velocity.norm();
    solve_params.end_heading = 0;
    solve_params.end_magnitude = 0;

    path.solve_coeffs(solve_params);
}
