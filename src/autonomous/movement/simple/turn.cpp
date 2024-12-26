#include "autonomous/movement/simple/turn.h"
#include "essential.h"

using namespace movement;

SimpleResult simple::turn_towards_tick(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid)
{
    const float diff = robot::angular_diff(angle, params.reversed);
    const float ctrl = in_place_pid.get(diff);
    robot::velo(ctrl, -ctrl);
    return { ExitCode::SUCCESS, diff, 0 };
}

SimpleResult simple::turn_towards_cancellable(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& pid,
    volatile bool& cancel_ref)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (fabs(robot::angular_diff(angle, params.reversed)) > params.exit_threshold) {
        if (cancel_ref) {
            return { ExitCode::CANCELLED, last_tick.error, __timediff(start) };
        }

        if (pros::millis() - start >= params.timeout) {
            return { ExitCode::TIMEOUT, last_tick.error, __timediff(start) };
        }

        last_tick = turn_towards_tick(angle, params, pid);

        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, __timediff(start) };
        }

        pros::delay(params.delay);
    }

    return { ExitCode::SUCCESS, last_tick.error, __timediff(start) };
}

SimpleResult simple::turn_towards(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& pid)
{
    bool dummy_ref = false;
    return turn_towards_cancellable(angle, params, pid, dummy_ref);
}

Future<SimpleResult> simple::turn_towards_async(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& pid)
{
    Future<SimpleResult> future;
    pros::Task task([&future, angle, &pid, &params]() {
        future.set_value(turn_towards_cancellable(
            angle, params, pid, 
            future.get_state()->cancelled
        ));
    });
    return future;
}

// Useful overload for turning towards a point

SimpleResult simple::turn_towards_tick(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid)
{
    const float angle = atan2f(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_tick(angle, params, in_place_pid);
}

SimpleResult simple::turn_towards_cancellable(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid,
    volatile bool& cancel_ref)
{
    const float angle = atan2(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_cancellable(angle, params, pid, cancel_ref);
}

SimpleResult simple::turn_towards(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid)
{
    const float angle = atan2f(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards(angle, params, in_place_pid);
}

Future<SimpleResult> simple::turn_towards_async(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid)
{
    const float angle = atan2f(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_async(angle, params, pid);
}