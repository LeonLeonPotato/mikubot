#include "autonomous/movement/pure_pursuit.h"
#include "essential.h"

using namespace movement;

float PurePursuit::func(pathing::BasePath& path, float radius, float t) const {
    return (robot::pos - path.compute(t)).norm() - radius;
}

float PurePursuit::deriv(pathing::BasePath& path, float t) const {
    Eigen::Vector2f diff = robot::pos - path.compute(t);
    return diff.dot(path.compute(t, 1)) / diff.norm();
}

Eigen::VectorXf PurePursuit::vec_func(pathing::BasePath& path, float radius, Eigen::VectorXf& t) const {
    return (path.compute(t).colwise() - robot::pos).colwise().norm().array() - radius;
}

Eigen::VectorXf PurePursuit::vec_deriv(pathing::BasePath& path, Eigen::VectorXf& t) const {
    const Eigen::Matrix2Xf rel = path.compute(t).colwise() - robot::pos;
    return rel.cwiseProduct(path.compute(t, 1)).colwise().sum().cwiseQuotient(rel.colwise().norm());
}

TickResult PurePursuit::tick(
    pathing::BasePath& path, const MovementParams& params, controllers::PID& pid, 
    const solvers::FunctionGroup& funcs, float t) const 
{
    TickResult result;

    Eigen::Vector2f& dest = path.points.back();

    bool end_of_path = fabs(t - path.maxt()) < 0.0001; // is old t the end of the path?
    if (!end_of_path) { // lets update t and store it in result
        std::tie(result.t, result.error) = compute_updated_t(path, params, funcs, t);
        end_of_path = fabs(result.t - path.maxt()) < 0.0001;
    } else { // update result to reflect end of path
        result.t = path.maxt();
        result.error = funcs.funcs[0](result.t); // should give raw error
    }

    const bool pathing_error = fabs(result.error) > params.recomputation_threshold || result.t < 0;

    if ((pathing_error || params.force_recomputation != RecomputationLevel::NONE) && !end_of_path) {
        // Error is too high, or intersection not found, or some mode of always recompute is enabled
        // We must be recomputing the time
        result.recomputation_level = RecomputationLevel::TIME;

        if (pathing_error || params.force_recomputation == RecomputationLevel::PATH_AND_TIME) {
            // Error is too high or intersection still not found, or force path and time recomputation is enabled
            result.recomputation_level = RecomputationLevel::PATH_AND_TIME;
            int goal = std::clamp((int) ceil(t), 1, (int) roundf(path.maxt())); // prevent floating point error
            recompute_path(path, goal);
        }

        std::tie(result.t, result.error) = compute_initial_t(path, params, funcs);

        if (fabs(result.error) > params.recomputation_threshold || result.t < 0) {
            // Error is still too high or intersection still not found
            // We have to abort since compute_initial_t is deterministic, so we can't just call it again
            result.code = ExitCode::RECOMPUTATION_ERROR;
            return result;
        }
    }

    // Anyways, we have a valid new t now (or we have aborted), so lets do speed calculations
    Eigen::Vector2f res = path.compute(result.t);
    float speed = fmin(robot::distance(dest) * params.distance_coeff, params.max_base_speed);
    float turn = pid.get(robot::angular_diff(res, params.reversed));
    if (params.reversed) speed = -speed;

    // Move the robot
    robot::volt(
        (int) (speed + turn),
        (int) (speed - turn)
    );

    result.code = ExitCode::SUCCESS;
    return result;
}

MovementResult PurePursuit::follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const MovementParams& params,
    controllers::PID& pid) const 
{
    MovementResult result;

    int start_t = pros::millis();

    // Create function group with binds to member functions
    float radius = ((const PurePursuitParams&) params).radius;
    solvers::FunctionGroup funcs = {
        {
            std::bind(&PurePursuit::func, this, std::ref(path), radius, std::placeholders::_1),
            std::bind(&PurePursuit::deriv, this, std::ref(path), std::placeholders::_1)
        },
        {
            std::bind(&PurePursuit::vec_func, this, std::ref(path), radius, std::placeholders::_1),
            std::bind(&PurePursuit::vec_deriv, this, std::ref(path), std::placeholders::_1)
        }
    };

    // Compute our initial path with the goal being the first point in the path (discounting robot)
    recompute_path(path, 1);
    std::tie(result.t, result.error) = compute_initial_t(path, params, funcs);

    while (robot::distance(path.points.back()) > params.final_threshold) {
        if (cancel_ref) { // cancel handling
            result.code = ExitCode::CANCELLED;
            break;
        }

        if (pros::millis() - start_t >= params.timeout) { // timeout handling
            result.code = ExitCode::TIMEOUT;
            break;
        }

        if (robot::distance(path.points.back()) <= radius) result.t = path.maxt(); // we are capable of reaching the end
        TickResult tick_result = tick(path, params, pid, funcs, result.t);

        if (tick_result.code != ExitCode::SUCCESS) { // Any error that happened during the tick = exit
            result.code = tick_result.code;
            break;
        }

        result.t = tick_result.t;
        result.error = tick_result.error;
        if (tick_result.recomputation_level != RecomputationLevel::NONE) {
            result.num_time_recomputations++;
            if (tick_result.recomputation_level == RecomputationLevel::PATH_AND_TIME) result.num_path_recomputations++;
        }

        pros::delay(params.delay);
    }

    if (result.code == ExitCode::TBD) result.code = ExitCode::SUCCESS; // if we reached here and code has not been set, we are successful
    result.time_taken_ms = pros::millis() - start_t;

    return result;
}