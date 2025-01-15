#include "autonomous/movement/simple/turn.h"
#include "Eigen/src/Core/Matrix.h"
#include "essential.h"
#include "simpletils.h"

using namespace movement;

DEFINE_TICK(turn_towards, controllers::PID&, const float angle)
{
    const float diff = robot::angular_diff(angle, params.reversed);
    const float ctrl = pids.get(diff);
    robot::velo(ctrl, -ctrl);
    return { ExitCode::SUCCESS, diff, 0 };
}

DEFINE_CANCELLABLE(turn_towards, controllers::PID&, const float angle)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (fabs(robot::angular_diff(angle, params.reversed)) > params.angular_exit_threshold) {
        if (cancel_ref) {
            return { ExitCode::CANCELLED, last_tick.error, __timediff(start) };
        }

        if (pros::millis() - start >= params.timeout) {
            return { ExitCode::TIMEOUT, last_tick.error, __timediff(start) };
        }

        last_tick = turn_towards_tick(angle, params, pids);

        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, __timediff(start) };
        }

        pros::delay(params.delay);
    }

    return { ExitCode::SUCCESS, last_tick.error, __timediff(start) };
}

DEFINE_STANDARD(turn_towards, controllers::PID&, const float angle)
{
    bool dummy_ref = false;
    return turn_towards_cancellable(angle, params, pids, dummy_ref);
}

DEFINE_ASYNC(turn_towards, controllers::PID&, const float angle)
{
    Future<SimpleResult> future;
    pros::Task task([&future, angle, &pids, &params]() {
        future.set_value(turn_towards_cancellable(
            angle, params, pids, 
            future.get_state()->cancelled
        ));
    });
    return future;
}

// Useful overload for turning towards a point

DEFINE_TICK(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2f(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_tick(angle, params, pids);
}

DEFINE_CANCELLABLE(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_cancellable(angle, params, pids, cancel_ref);
}

DEFINE_STANDARD(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2f(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards(angle, params, pids);
}

DEFINE_ASYNC(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2f(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_async(angle, params, pids);
}