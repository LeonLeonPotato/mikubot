#include "autonomous/movement/pure_pursuit.h"
#include "autonomous/movement/base_movement.h"
#include "essential.h"

using namespace movement;

// TODO: add curvature based adaptive base speed
float pure_pursuit::follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, float radius,
                                    solvers::func_t func, solvers::func_t deriv, 
                                    solvers::func_vec_t vec_func, solvers::func_vec_t vec_deriv, 
                                    int iterations) 
{
    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::Vector2f& last_point = path.points.back();

    float error;
    switch (path.get_solver()) {
        case solvers::Solver::Newton:
            std::tie(t, error) = utils::compute_updated_t_newton(path, func, deriv, t, iterations);
            break;
        case solvers::Solver::Secant:
            std::tie(t, error) = utils::compute_updated_t_secant(path, func, t, iterations);
            break;
        default:
            return -1;
    }

    if ((t < 0 || error > 1) && fabs(t - path.points.size() + 1) > 1e-3) {
        int goal = (int) ceilf(t);
        goal = std::clamp(goal, 0, (int) path.points.size() - 1);
        std::tie(t, error) = utils::recompute_path(path, vec_func, vec_deriv, path.get_solver(), goal);
    }

    if (error > 1) {
        return -1;
    }

    Eigen::Vector2f res = path.compute(t);

    float dtheta = robot::angular_diff(res);
    pid.register_error(fabs(dtheta));

    float dist = robot::distance(last_point);
    dist = fmin(dist * movement::variables::distance_coeff, radius) / radius * 127;

    float ctrl = pid.get();

    robot::volt(
        (int) (dist + ctrl * dtheta),
        (int) (dist - ctrl * dtheta)
    );
    return t;
}

float pure_pursuit::follow_path(pathing::BasePath& path,
                float radius,
                controllers::PID* pid,
                int iterations, long long timeout)
{
    bool delete_pid = pid == nullptr;

    if (delete_pid) {
        pid = new controllers::PID();
        init_pid(*pid);
    }

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
    float t = utils::recompute_path(path, vec_func, vec_deriv, path.get_solver(), 1, false).first;
    while (true) {
        point.noalias() = Eigen::Vector2f(robot::x, robot::y);
        t = pure_pursuit::follow_path_tick(
            path, *pid, t, radius, 
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

