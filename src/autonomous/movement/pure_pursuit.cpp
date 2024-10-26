#include "autonomous/movement/pure_pursuit.h"
#include "essential.h"

using namespace movement;

// TODO: add curvature based adaptive base speed
std::pair<float, float> 
pure_pursuit::follow_path_tick(pathing::BasePath& path, pathing::BaseParams& solve_params,
    controllers::PID& pid, float t, float radius,
    solvers::func_t func, solvers::func_t deriv, 
    solvers::func_vec_t vec_func, solvers::func_vec_t vec_deriv, 
    MovementResult& result, int iterations) 
{
    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::Vector2f& last_point = path.points.back();

    float error;
    std::tie(t, error) = utils::compute_updated_t(path.get_solver(), path, func, deriv, t, iterations);

    if ((t < 0 || error > variables::recomputation_error) && fabs(t - path.points.size() + 1) > 1e-3) {
        #ifdef MIKU_VERBOSE
            printf("Recomputing path | Error: %f, t: %f, t_left: %f\n", error, t, fabs(t - path.points.size() + 1));
        #endif
        int goal = (int) ceilf(t);
        goal = std::clamp(goal, 1, (int) path.points.size() - 1);
        utils::recompute_path(path, solve_params, goal);
        std::tie(t, error) = utils::compute_initial_t(path.get_solver(), path, vec_func, vec_deriv);
        result.num_recomputations++;
    }

    if (error > variables::recomputation_error) {
        result.code = ExitCodes::RECOMPUTATION_ERROR;
        return {-1, -1};
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
    return {t, error};
}

MovementResult pure_pursuit::follow_path(pathing::BasePath& path, pathing::BaseParams& params,
                float radius,
                controllers::PID* pid,
                int iterations, int timeout)
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

    MovementResult result;

    int start = pros::millis();
    utils::recompute_path(path, params, 1);
    float t = utils::compute_initial_t(path.get_solver(), path, vec_func, vec_deriv).first;

    while (robot::distance(path.points.back()) > 5) {
        point = Eigen::Vector2f(robot::x, robot::y);
        auto [new_t, error] = pure_pursuit::follow_path_tick(
            path, params, *pid, t, radius,
            func, deriv, 
            vec_func, vec_deriv, 
            result, iterations
        );

        result.error = error;
        if (new_t < 0) break;
        result.t = new_t;

        if (pros::millis() - start > timeout) {
            result.code = ExitCodes::TIMEOUT;
            break;
        }
        t = new_t;
    }

    if (result.code == ExitCodes::TBD) result.code = ExitCodes::SUCCESS;
    result.time_taken_ms = pros::millis() - start;

    return result;
}

Future<MovementResult> follow_path_async(pathing::BasePath& path, pathing::BaseParams& solve_params,
    float radius,
    controllers::PID* pid = nullptr,
    int iterations, int timeout)
{
    Future<MovementResult> f;

    auto task = [&f, &path, &solve_params, radius, pid, iterations, timeout]() {
        MovementResult result = pure_pursuit::follow_path(path, solve_params, radius, pid, iterations, timeout);
        f.set_value(result);
    };

    pros::Task t(task);

    return f;
}