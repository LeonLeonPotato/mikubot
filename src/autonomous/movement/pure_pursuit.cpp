#include "autonomous/movement/pure_pursuit.h"
#include "autonomous/movement/base_movement.h"
#include "essential.h"

using namespace movement;

std::pair<float, float> 
compute_initial_t_newton(pathing::BasePath& path, solvers::func_vec_t func, solvers::func_vec_t deriv) 
{
    Eigen::VectorXf guesses = Eigen::VectorXf(32);
    guesses.LinSpaced(32, 0.05, path.points.size() - 1.05);
    return solvers::newton_vec(func, deriv, guesses, 0, path.points.size() - 1, 15);
}

std::pair<float, float> 
compute_initial_t_secant(pathing::BasePath& path, solvers::func_vec_t func) {
    Eigen::VectorXf guesses = Eigen::VectorXf(32);
    guesses.LinSpaced(32, 0.05, path.points.size() - 1.10);

    Eigen::VectorXf guesses2 = Eigen::VectorXf(32);
    guesses2.LinSpaced(32, 0.10, path.points.size() - 1.05);

    return solvers::secant_vec(func, guesses, guesses2, 0, path.points.size() - 1, 15);
}

inline std::pair<float, float> 
compute_updated_t_newton(pathing::BasePath& path, solvers::func_t func, solvers::func_t deriv, float t, int iterations) {
    return solvers::newton_single(func, deriv, t, t, path.points.size() - 1, iterations);
}

inline std::pair<float, float> 
compute_updated_t_secant(pathing::BasePath& path, solvers::func_t func, float t, int iterations) {
    return solvers::secant_single(func, t, t + 0.05, t, path.points.size() - 1, iterations);
}

std::pair<float, float>
recompute_path(pathing::BasePath& path, int goal_i, solvers::func_vec_t func, solvers::func_vec_t deriv) {
    if (goal_i > 1) {
        for (int i = goal_i; i < path.points.size(); i++) {
            path.points[i - goal_i + 1] = path.points[i];
        }
        for (int i = 0; i < goal_i - 1; i++) {
            path.points.pop_back();
        }
    }

    path.points[0] = Eigen::Vector2f(robot::x, robot::y);

    if (path.need_solve()) {
        pathing::BaseParams params;
        params.start_heading = robot::theta;
        params.start_magnitude = 10;
        params.end_heading = 0;
        params.end_magnitude = 0;
        path.solve_coeffs(params);
    }

    switch (path.get_solver()) {
        case solvers::Solver::Newton:
            return compute_initial_t_newton(path, func, deriv);
        case solvers::Solver::Secant:
            return compute_initial_t_secant(path, func);
        default:
            return {0, 0};
    }
}

// TODO: add curvature based adaptive base speed
float pure_pursuit::follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, float radius,
                        solvers::func_t func, solvers::func_t deriv, 
                        solvers::func_vec_t vec_func, solvers::func_vec_t vec_deriv, 
                        int iterations) 
{
    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);

    float error;
    switch (path.get_solver()) {
        case solvers::Solver::Newton:
            std::tie(t, error) = compute_updated_t_newton(path, func, deriv, t, iterations);
            break;
        case solvers::Solver::Secant:
            std::tie(t, error) = compute_updated_t_secant(path, func, t, iterations);
            break;
        default:
            return -1;
    }

    if ((t < 0 || error > 1) && fabs(t - path.points.size() + 1) > 1e-3) {
        int goal = (int) ceilf(t);
        goal = std::clamp(goal, 0, (int) path.points.size() - 1);
        std::tie(t, error) = recompute_path(path, goal, vec_func, vec_deriv);
    }

    if (error > 1) {
        return -1;
    }

    Eigen::Vector2f res = path.compute(t);

    float dtheta = robot::angular_diff(res);
    pid.register_error(fabs(dtheta));

    float dist = robot::distance(res);
    dist = fmin(dist * movement::variables::distance_coeff, radius) / radius * 127;

    float ctrl = pid.get();

    robot::volt(
        (int) (dist + ctrl * dtheta),
        (int) (dist - ctrl * dtheta)
    );
    return t;
}

float pure_pursuit::follow_path(pathing::BasePath& path, float radius, int iterations, long long timeout) {
    controllers::PID pid; init_pid(pid);
    Eigen::Vector2f point;

    // VERY IMPORTANT TYPE YOUR LAMBDAS!!!!!!!!
    auto func = [&path, &radius, &point](float t) -> float { 
        return (point - path.compute(t)).norm() - radius; 
    };
    auto deriv = [&path, &radius, &point](float t) -> float {
        Eigen::Vector2f diff = point - path.compute(t);
        return diff.dot(path.compute(t, 1)) / diff.norm();
    };
    auto vec_func = [&path, &radius, &point](Eigen::VectorXf& t) -> Eigen::VectorXf {
        return (path.compute(t).colwise() - point).colwise().norm().array() - radius;
    };
    auto vec_deriv = [&path, &radius, &point](Eigen::VectorXf& t) -> Eigen::VectorXf {
		const Eigen::Matrix2Xf rel = path.compute(t).colwise() - point;
		return rel.cwiseProduct(path.compute(t, 1)).colwise().sum().cwiseQuotient(rel.colwise().norm());
	};

    long long start = pros::millis();
    float t;
    while (true) {
        point = Eigen::Vector2f(robot::x, robot::y);
        t = pure_pursuit::follow_path_tick(
            path, pid, 0, radius, 
            func, deriv, vec_func, vec_deriv,
            iterations
        );

        if (t < 0 || pros::millis() - start > timeout) {
            break;
        }
    }

    robot::brake();

    return t;
}

