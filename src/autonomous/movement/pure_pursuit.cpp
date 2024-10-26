#include "autonomous/movement/pure_pursuit.h"
#include "essential.h"

using namespace movement;

float PurePursuit::func(float t) const {
    return (Eigen::Vector2f(robot::x, robot::y) - path.compute(t)).norm() - radius;
}

float PurePursuit::deriv(float t) const {
    Eigen::Vector2f diff = Eigen::Vector2f(robot::x, robot::y) - path.compute(t);
    return diff.dot(path.compute(t, 1)) / diff.norm();
}

Eigen::VectorXf PurePursuit::vec_func(Eigen::VectorXf& t) const {
    return (path.compute(t).colwise() - Eigen::Vector2f(robot::x, robot::y)).colwise().norm().array() - radius;
}

Eigen::VectorXf PurePursuit::vec_deriv(Eigen::VectorXf& t) const {
    const Eigen::Matrix2Xf rel = path.compute(t).colwise() - Eigen::Vector2f(robot::x, robot::y);
    return rel.cwiseProduct(path.compute(t, 1)).colwise().sum().cwiseQuotient(rel.colwise().norm());
}

TickResult PurePursuit::tick(float t) {
    TickResult result;

    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::Vector2f& dest = path.points.back();

    std::tie(result.t, result.error) = utils::compute_updated_t(get_solver(), *this, t);

    if (result.error > recomputation_error || result.t < 0) {
        result.recomputed = true;

        int goal = std::clamp((int) ceil(t), 0, (int) maxt());
        utils::recompute_path(*this, goal);
        std::tie(result.t, result.error) = utils::compute_initial_t(get_solver(), *this);

        if (result.error > recomputation_error || result.t < 0) {
            result.code = ExitCode::RECOMPUTATION_ERROR;
            return result;
        }
    }

    Eigen::Vector2f res = path.compute(t);

    float dtheta = robot::angular_diff(res);
    pid.register_error(fabs(dtheta));

    float dist = robot::distance(dest);
    dist = fmin(dist * distance_coeff, radius) / radius * 127;

    float ctrl = pid.get();

    robot::volt(
        (int) (dist + ctrl * dtheta),
        (int) (dist - ctrl * dtheta)
    );

    result.code = ExitCode::SUCCESS;
    return result;
}

MovementResult PurePursuit::follow_path_cancellable(bool& cancel_ref) {
    MovementResult result;
    result.code = ExitCode::SUCCESS;

    int start_t = pros::millis();

    utils::recompute_path(*this, 1);
    std::tie(result.t, result.error) = utils::compute_initial_t(get_solver(), *this);

    while (robot::distance(path.points.back()) > final_threshold) {
        if (cancel_ref) {
            result.code = ExitCode::CANCELLED;
            break;
        }

        TickResult tick_result = tick(result.t);

        if (tick_result.code != ExitCode::SUCCESS) {
            result.code = tick_result.code;
            break;
        }

        if (pros::millis() - start_t > timeout) {
            result.code = ExitCode::TIMEOUT;
            break;
        }

        result.t = tick_result.t;
        result.error = tick_result.error;
        if (tick_result.recomputed) result.num_recomputations++;

        pros::delay(20);
    }

    result.time_taken_ms = pros::millis() - start_t;

    return result;
}