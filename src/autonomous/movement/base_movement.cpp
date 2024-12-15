#include "autonomous/movement.h"
#include "essential.h"

using namespace movement;

std::pair<float, float> utils::compute_initial_t(
    const pathing::BasePath& path, 
    const float guesses,
    const NumericalRecomputationParams& params) 
{
    switch (params.solver) {
        case solvers::Solver::Newton: {
            Eigen::VectorXf guess = Eigen::VectorXf::LinSpaced(guesses, 0.01, path.maxt() - 0.01);

            return solvers::newton_vec(params.funcs, guess, 0, path.maxt(), params.iterations, params.threshold);
            break;
        }
        case solvers::Solver::Secant: {
            Eigen::VectorXf t0 = Eigen::VectorXf::LinSpaced(guesses, 0.05, path.maxt() - 0.10);
            Eigen::VectorXf t1 = Eigen::VectorXf::LinSpaced(guesses, 0.10, path.maxt() - 0.05);

            return solvers::secant_vec(params.funcs, t0, t1, 0, path.maxt(), params.iterations, params.threshold);
            break;
        }
        default: {
            throw std::runtime_error("Invalid / Unsupported solver: " + std::to_string((int) params.solver));
        }
    }

    __builtin_unreachable();
}

std::pair<float, float> utils::compute_updated_t(
    pathing::BasePath& path, 
    const float t,
    const NumericalRecomputationParams& params)
{
    switch (params.solver) {
        case solvers::Solver::Newton: {
            return solvers::newton_single(params.funcs, t, 0, path.maxt(), params.iterations, params.threshold);
        }
        case solvers::Solver::Secant: {
            const float t1 = fmin(t + 1, path.maxt());
            const float t0 = t - ((int) (t1 == t)) * 0.1;
            return solvers::secant_single(params.funcs, t0, t1, 0, path.maxt(), params.iterations, params.threshold);
        }
        case solvers::Solver::GradientDescent: {
            return solvers::gradient_descent_single(params.funcs, t, 0, path.maxt(), params.step_size, params.iterations);
        }
        default: {
            throw std::runtime_error("Invalid / Unsupported solver: " + std::to_string((int) params.solver));
        }
    }

    __builtin_unreachable();
}

void utils::recompute_path(
    pathing::BasePath& path, 
    const path_solver_t path_solver,    
    const int goal_i)
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
}

void utils::solve_path_default(pathing::BasePath& path) {
    pathing::BaseParams solve_params;

    solve_params.start_heading = robot::theta;
    solve_params.start_magnitude = robot::velocity.norm();
    solve_params.end_heading = 0;
    solve_params.end_magnitude = 0;

    path.solve_coeffs(solve_params);
}
