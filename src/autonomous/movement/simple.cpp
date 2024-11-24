#include "autonomous/movement/simple.h"
#include "essential.h"

using namespace movement;

void simple::init_generic_pid(controllers::PID& pid) {
    pid.kp = 4000;
    pid.ki = 0;
    pid.kd = 50;
    pid.disable_integral_limit = infinity();
    pid.integral_limit = infinity();
    pid.sign_switch_reset = true;
}

TickResult simple::turn_towards_tick(float angle, controllers::PID& pid,
    int timeout, float threshold)
{
    float diff = robot::angular_diff(angle);
    int ctrl = (int) pid.get(diff);
    robot::volt(ctrl, -ctrl);
    return { ExitCode::SUCCESS, 0, diff, RecomputationLevel::NONE };
}

MovementResult simple::turn_towards_cancellable(float angle, controllers::PID& pid,
    bool& cancel_ref, int timeout, float threshold)
{
    MovementResult result;

    int start = pros::millis();
    while (true) {
        if (cancel_ref) {
            result.code = ExitCode::CANCELLED;
            break;
        }

        float diff = robot::angular_diff(angle);
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

MovementResult simple::turn_towards_cancellable(float angle, bool& cancel_ref, int timeout, float threshold)
{
    controllers::PID pid;
    init_generic_pid(pid);
    return turn_towards_cancellable(angle, pid, cancel_ref, timeout, threshold);
}

MovementResult simple::turn_towards(float angle, controllers::PID& pid, int timeout, float threshold)
{
    bool cancel = false;
    return turn_towards_cancellable(angle, pid, cancel, timeout, threshold);
}

MovementResult simple::turn_towards(float angle, int timeout, float threshold)
{
    controllers::PID pid;
    init_generic_pid(pid);
    return turn_towards(angle, pid, timeout, threshold);
}

Future<MovementResult> simple::turn_towards_async(float angle, controllers::PID& pid, int timeout, float threshold)
{
    Future<MovementResult> future;
    pros::Task task([angle, &pid, timeout, threshold, &future]() {
        future.set_value(turn_towards(angle, pid, timeout, threshold));
    });
    return future;
}

Future<MovementResult> simple::turn_towards_async(float angle, int timeout, float threshold)
{
    controllers::PID pid; init_generic_pid(pid);
    Future<MovementResult> future;
    pros::Task task([angle, &pid, timeout, threshold, &future]() {
        future.set_value(turn_towards(angle, pid, timeout, threshold));
    });
    return future;
}

TickResult simple::go_to_tick(const Eigen::Vector2f& point, controllers::PID& pid,
    float distance_coeff, int max_speed, int timeout, float threshold)
{
    float dist = robot::distance(point);
    float speed = fmin(dist * distance_coeff, 12000.0f);
    float dtheta = robot::angular_diff(point);
    float ctrl = pid.get(dtheta);
    robot::volt(dist + ctrl, dist - ctrl);

    return { ExitCode::SUCCESS, 0, dist, RecomputationLevel::NONE };
}

MovementResult simple::go_to_cancellable(const Eigen::Vector2f& point, controllers::PID& pid,
    bool& cancel_ref, float distance_coeff, int max_speed, int timeout, float threshold)
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

        auto res = go_to_tick(point, pid, distance_coeff, max_speed, timeout, threshold);
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