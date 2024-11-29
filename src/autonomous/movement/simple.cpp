#include "autonomous/movement/simple.h"
#include "essential.h"

using namespace movement;

////// Turn in place

TickResult&& simple::turn_towards_tick(const float angle, controllers::PID& in_place_pid,
    const int timeout, const float threshold)
{
    const float diff = robot::angular_diff(angle);
    const float ctrl = in_place_pid.get(diff);
    robot::volt(ctrl, -ctrl);
    return { ExitCode::SUCCESS, 0, diff, RecomputationLevel::NONE };
}

MovementResult&& simple::turn_towards_cancellable(float angle, controllers::PID& pid,
    bool& cancel_ref, int timeout, float threshold)
{
    MovementResult result;

    const int start = pros::millis();
    while (true) {
        if (cancel_ref) {
            result.code = ExitCode::CANCELLED;
            break;
        }

        const float diff = robot::angular_diff(angle);
        if (fabs(diff) < threshold) {
            result.code = ExitCode::SUCCESS;
            break;
        }

        if (pros::millis() - start >= timeout) {
            result.code = ExitCode::TIMEOUT;
            break;
        }

        auto res = turn_towards_tick(angle, pid, timeout, threshold);
        result.error = res.error;

        pros::delay(20);
    }

    result.time_taken_ms = pros::millis() - start;

    return std::move(result);
}

MovementResult&& simple::turn_towards(const float angle, controllers::PID& pid,
    const int timeout, const float threshold)
{
    bool dummy_ref = false;
    return turn_towards_cancellable(angle, pid, dummy_ref, timeout, threshold);
}

Future<MovementResult> simple::turn_towards_async(const float angle, controllers::PID& pid, 
    const int timeout, const float threshold)
{
    Future<MovementResult> future;
    pros::Task task([angle, &pid, timeout, threshold, &future]() {
        future.set_value(turn_towards(angle, pid, timeout, threshold));
    });
    return future;
}

////// Swing to position

TickResult simple::swing_to_tick(const Eigen::Vector2f& point, PIDGroup pids, 
    const bool reversed, const int timeout, const float max_base_speed, const float threshold)
{
    const float dist = robot::distance(point);
    float speed = fmin(pids.linear.get(dist), max_base_speed);
    float turn = pids.angular.get(robot::angular_diff(point, reversed));
    if (reversed) speed = -speed;
    robot::volt(speed + turn, speed - turn);

    return { ExitCode::SUCCESS, 0, dist, RecomputationLevel::NONE };
}

MovementResult simple::swing_to_cancellable(const Eigen::Vector2f& point, PIDGroup pids,
    volatile bool& cancel_ref, const bool reversed, const int timeout, const float max_base_speed, const float threshold)
{
    MovementResult result;

    int start = pros::millis();
    while (true) {
        if (cancel_ref) {
            result.code = ExitCode::CANCELLED;
            break;
        }

        float dist = robot::distance(point);
        if (dist < threshold) {
            result.code = ExitCode::SUCCESS;
            break;
        }

        if (pros::millis() - start > timeout) {
            result.code = ExitCode::TIMEOUT;
            break;
        }

        auto res = swing_to_tick(point, pids, max_base_speed, timeout, threshold);
        result.error = res.error;

        pros::delay(20);
    }

    result.time_taken_ms = pros::millis() - start;

    return result;
}

MovementResult simple::go_to_cancellable(const Eigen::Vector2f& point, bool& cancel_ref,
    float distance_coeff, int max_speed, int timeout, float threshold)
{
    controllers::PID pid; init_generic_pid(pid);
    return go_to_cancellable(point, pid, cancel_ref, distance_coeff, max_speed, timeout, threshold);
}

MovementResult simple::go_to(const Eigen::Vector2f& point, controllers::PID& pid,
    float distance_coeff, int max_speed, int timeout, float threshold)
{
    bool cancel = false;
    return go_to_cancellable(point, pid, cancel, distance_coeff, max_speed, timeout, threshold);
}

MovementResult simple::go_to(const Eigen::Vector2f& point,
    float distance_coeff, int max_speed, int timeout, float threshold)
{
    bool cancel = false;
    controllers::PID pid; init_generic_pid(pid);
    return go_to_cancellable(point, pid, cancel, distance_coeff, max_speed, timeout, threshold);
}

Future<MovementResult> simple::go_to_async(const Eigen::Vector2f& point, controllers::PID& pid,
    float distance_coeff, int max_speed, int timeout, float threshold)
{
    Future<MovementResult> future;
    pros::Task task([point, &pid, distance_coeff, max_speed, timeout, threshold, &future]() {
        future.set_value(go_to(point, pid, distance_coeff, max_speed, timeout, threshold));
    });
    return future;
}

Future<MovementResult> simple::go_to_async(const Eigen::Vector2f& point,
    float distance_coeff, int max_speed, int timeout, float threshold)
{
    controllers::PID pid; init_generic_pid(pid);
    Future<MovementResult> future;
    pros::Task task([point, &pid, distance_coeff, max_speed, timeout, threshold, &future]() {
        future.set_value(go_to(point, pid, distance_coeff, max_speed, timeout, threshold));
    });
    return future;
}