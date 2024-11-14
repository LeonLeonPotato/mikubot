#include "autonomous/movement/pure_pursuit.h"
#include "essential.h"

using namespace movement;

float PurePursuit::func(float t) const {
    return (robot::pos - path.compute(t)).norm() - radius;
}

float PurePursuit::deriv(float t) const {
    Eigen::Vector2f diff = robot::pos - path.compute(t);
    return diff.dot(path.compute(t, 1)) / diff.norm();
}

Eigen::VectorXf PurePursuit::vec_func(Eigen::VectorXf& t) const {
    return (path.compute(t).colwise() - robot::pos).colwise().norm().array() - radius;
}

Eigen::VectorXf PurePursuit::vec_deriv(Eigen::VectorXf& t) const {
    const Eigen::Matrix2Xf rel = path.compute(t).colwise() - robot::pos;
    return rel.cwiseProduct(path.compute(t, 1)).colwise().sum().cwiseQuotient(rel.colwise().norm());
}

TickResult PurePursuit::tick(float t) {
    TickResult result;

    Eigen::Vector2f& dest = path.points.back();

    std::tie(result.t, result.error) = compute_updated_t(get_solver(), t);
    bool end_of_path = fabs(result.t - maxt()) < 0.0001;

    if ((fabs(result.error) > params.recomputation_error || result.t < 0 || params.always_recompute) && !end_of_path) {
        result.recomputed = true;

        int goal = std::clamp((int) ceil(t), 1, (int) roundf(maxt())); // prevent floating point error
        recompute_path(goal);
        std::tie(result.t, result.error) = compute_initial_t(get_solver());

        if (fabs(result.error) > params.recomputation_error || result.t < 0) {
            result.code = ExitCode::RECOMPUTATION_ERROR;
            return result;
        }
    }

    Eigen::Vector2f res = path.compute(result.t);
    float speed = fmin(robot::distance(dest) * params.distance_coeff, params.max_base_speed);
    float ctrl = pid.get(robot::angular_diff(res, params.reversed));
    if (params.reversed) speed = -speed;

    robot::volt(
        (int) (speed + ctrl),
        (int) (speed - ctrl)
    );

    result.code = ExitCode::SUCCESS;
    return result;
}

MovementResult PurePursuit::follow_path_cancellable(bool& cancel_ref) {
    MovementResult result;

    int start_t = pros::millis();

    recompute_path(1);
    std::tie(result.t, result.error) = compute_initial_t(get_solver());

    while (robot::distance(path.points.back()) > params.final_threshold) {
        if (cancel_ref) {
            result.code = ExitCode::CANCELLED;
            break;
        }

        if (pros::millis() - start_t >= params.timeout) {
            result.code = ExitCode::TIMEOUT;
            break;
        }

        if (robot::distance(path.points.back()) <= radius) result.t = maxt();
        TickResult tick_result = tick(result.t);

        if (tick_result.code != ExitCode::SUCCESS) {
            result.code = tick_result.code;
            break;
        }

        result.t = tick_result.t;
        result.error = tick_result.error;
        if (tick_result.recomputed) result.num_recomputations++;

        pros::delay(20);
    }

    if (result.code == ExitCode::TBD) result.code = ExitCode::SUCCESS;
    result.time_taken_ms = pros::millis() - start_t;

    return result;
}