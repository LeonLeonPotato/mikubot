#include "autonomous/movement/pure_pursuit.h"
#include "autonomous/controllers.h"
#include "essential.h"

using namespace movement::purepursuit;
using namespace movement;

static float func(pathing::BasePath& path, float radius, float t) {
    return (robot::pos - path.compute(t)).norm() - radius;
}

static float deriv(pathing::BasePath& path, float t) {
    Eigen::Vector2f diff = robot::pos - path.compute(t);
    return diff.dot(path.compute(t, 1)) / diff.norm();
}

static Eigen::ArrayXf vec_func(pathing::BasePath& path, float radius, const Eigen::ArrayXf& t) {
    return (path.compute(t).colwise() - robot::pos).colwise().norm().array() - radius;
}

static Eigen::ArrayXf vec_deriv(pathing::BasePath& path, const Eigen::ArrayXf& t) {
    const Eigen::MatrixX2f rel = path.compute(t).colwise() - robot::pos;
    return rel.cwiseProduct(path.compute(t, 1)).colwise().sum().cwiseQuotient(rel.colwise().norm());
}

static PurePursuitResult tick(
    pathing::BasePath& path, const PurePursuitParams& params, PIDGroup pids, 
    path_solver_t path_recomputer,
    const NumericalRecomputationParams& recomp_params,
    const NumericalRecomputationParams& update_params, 
    float t) 
{
    const Eigen::Vector2f& dest = path.points.back();

    PurePursuitResult result;

    bool end_of_path = fabs(t - path.maxt()) < 0.0001; // is old t the end of the path?
    if (!end_of_path) { // lets update t and store it in result
        std::tie(result.t, result.error) = utils::compute_updated_t(path, t, update_params);
        end_of_path = fabs(result.t - path.maxt()) < 0.0001;
    } else { // update result to reflect end of path
        result.t = path.maxt();
        result.error = recomp_params.funcs.funcs[0](result.t); // should give raw error
    }

    const bool pathing_error = fabs(result.error) > params.recomputation_threshold || result.t < 0;

    if ((pathing_error || params.force_recomputation != NumericalRecomputation::NONE) && !end_of_path) {
        // Error is too high, or intersection not found, or some mode of always recompute is enabled
        // We must be recomputing the time
        result.recomputation_level = NumericalRecomputation::TIME;

        if (pathing_error || params.force_recomputation == NumericalRecomputation::PATH_AND_TIME) {
            // Error is too high or intersection still not found, or force path and time recomputation is enabled
            result.recomputation_level = NumericalRecomputation::PATH_AND_TIME;
            int goal = std::clamp((int) ceil(t), 1, (int) roundf(path.maxt())); // prevent floating point error
            utils::recompute_path(path, path_recomputer, goal);
        }

        std::tie(result.t, result.error) = utils::compute_initial_t(path, params.recomputation_guesses, recomp_params);

        if (fabs(result.error) > params.recomputation_threshold || result.t < 0) {
            // Error is still too high or intersection still not found
            // We have to abort since compute_initial_t is deterministic, so we can't just call it again
            result.code = ExitCode::RECOMPUTATION_ERROR;
            return result;
        }
    }

    // Anyways, we have a valid new t now (or we have aborted), so lets do speed calculations
    const Eigen::Vector2f res = path.compute(result.t);
    const float res_dist = (res - robot::pos).norm();
    const float angular_diff = robot::angular_diff(res, params.reversed);
    float speed = pids.linear.get(res_dist);
    float turn = pids.angular.get(angular_diff);
    if (params.reversed) speed = -speed;
    if (params.use_cosine_scaling) speed *= fmax(0, cosf(angular_diff));
    speed = std::clamp(speed, -params.max_linear_speed, params.max_linear_speed);

    // Move the robot
    robot::velo(
        speed + turn,
        speed - turn
    );

    result.code = ExitCode::SUCCESS;
    return result;
}

PurePursuitResult purepursuit::follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const PurePursuitParams& params,
    const PIDGroup pids,
    path_solver_t path_solver) 
{
    PurePursuitResult result;

    const int start_t = pros::millis();

    // Create function group with binds to member functions
    float radius = ((const PurePursuitParams&) params).radius;
    solvers::FunctionGroup funcs = {
        {
            std::bind(&func, std::ref(path), radius, std::placeholders::_1),
            std::bind(&deriv, std::ref(path), std::placeholders::_1)
        },
        {
            std::bind(&vec_func, std::ref(path), radius, std::placeholders::_1),
            std::bind(&vec_deriv, std::ref(path), std::placeholders::_1)
        }
    };

    NumericalRecomputationParams update_params = {
        params.update_iterations,
        params.update_threshold,
        0,
        funcs,
        params.solver
    };

    NumericalRecomputationParams recomp_params = {
        params.recomputation_iterations,
        params.recomputation_threshold,
        0,
        funcs,
        params.solver
    };

    // Compute our initial path with the goal being the first point in the path (discounting robot)
    utils::recompute_path(path, path_solver, 1);
    std::tie(result.t, result.error) = utils::compute_initial_t(path, params.recomputation_guesses, recomp_params);

    while (robot::distance(path.points.back()) > params.exit_threshold) {
        if (cancel_ref) { // cancel handling
            result.code = ExitCode::CANCELLED;
            break;
        }

        if (pros::millis() - start_t >= params.timeout) { // timeout handling
            result.code = ExitCode::TIMEOUT;
            break;
        }

        if (robot::distance(path.points.back()) <= radius) result.t = path.maxt(); // we are capable of reaching the end
        PurePursuitResult tick_result = tick(path, params, pids, path_solver, recomp_params, update_params, result.t);

        if (tick_result.code != ExitCode::SUCCESS) { // Any error that happened during the tick = exit
            result.code = tick_result.code;
            break;
        }

        result.t = tick_result.t;
        result.error = tick_result.error;
        if (tick_result.recomputation_level != NumericalRecomputation::NONE) {
            result.num_time_recomputations++;
            if (tick_result.recomputation_level == NumericalRecomputation::PATH_AND_TIME) result.num_path_recomputations++;
        }

        pros::delay(params.delay);
    }

    if (result.code == ExitCode::TBD) result.code = ExitCode::SUCCESS; // if we reached here and code has not been set, we are successful
    result.time_taken_ms = pros::millis() - start_t;

    return result;
}

PurePursuitResult purepursuit::follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const float radius,
    const SimpleMovementParams& params,
    const PIDGroup pids,
    path_solver_t path_solver)
{
    PurePursuitParams new_params;
    new_params.copy_from(params);
    new_params.radius = radius;
    return follow_path_cancellable(cancel_ref, path, new_params, pids, path_solver);
}
