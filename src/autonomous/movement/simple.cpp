#include "autonomous/movement/simple.h"
#include "essential.h"

using namespace movement;

////// Turn in place

TickResult simple::turn_towards_tick(const float angle, controllers::PID& in_place_pid,
    const int timeout, const float threshold)
{
    const float diff = robot::angular_diff(angle);
    const float ctrl = in_place_pid.get(diff);
    robot::volt(ctrl, -ctrl);
    return { ExitCode::SUCCESS, 0, diff, RecomputationLevel::NONE };
}

MovementResult simple::turn_towards_cancellable(const float angle, controllers::PID& pid,
    volatile bool& cancel_ref, const int timeout, const float threshold)
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

    return result;
}

MovementResult simple::turn_towards(const float angle, controllers::PID& pid,
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
        future.set_value(turn_towards_cancellable(angle, pid, 
            future.get_state()->cancelled, timeout, threshold));
    });
    return future;
}

////// Swing to position

TickResult simple::swing_to_tick(const Eigen::Vector2f& point, PIDGroup pids, 
    const bool reversed, const int timeout, const float max_base_speed, const float threshold)
{
    const float dist = robot::distance(point);
    const float angle_diff = robot::angular_diff(point, reversed);
    float speed = fmin(pids.linear.get(dist), max_base_speed) * fmax(0, cosf(angle_diff));
    float turn = pids.angular.get(angle_diff);
    if (reversed) speed = -speed;
    // printf("Speed: %f, Turn: %f, Angular: %f, Reversed: %d, Dist: %f, Base: %f\n", speed, turn, angle_diff, reversed, dist, max_base_speed);
    robot::velo(speed + turn, speed - turn);

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

        auto res = swing_to_tick(point, pids, reversed, timeout, max_base_speed, threshold);
        result.error = res.error;

        pros::delay(20);
    }

    result.time_taken_ms = pros::millis() - start;

    return result;
}

MovementResult simple::swing_to(const Eigen::Vector2f& point, PIDGroup pids,
    const bool reversed, const int timeout, const float max_base_speed, const float threshold)
{
    bool cancel = false;
    return swing_to_cancellable(point, pids, cancel, reversed, timeout, max_base_speed, threshold);
}

Future<MovementResult> simple::swing_to_async(const Eigen::Vector2f& point, PIDGroup pids,
    const bool reversed, const int timeout, const float max_base_speed, const float threshold)
{
    Future<MovementResult> future;
    pros::Task task([point, &pids, reversed, timeout, max_base_speed, threshold, &future]() {
        future.set_value(swing_to_cancellable(point, pids, 
            future.get_state()->cancelled, reversed, timeout, max_base_speed, threshold));
    });
    return future;
}