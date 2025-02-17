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
    return { ExitCode::SUCCESS, 0, diff, 0 };
}

DEFINE_CANCELLABLE(turn_towards, controllers::PID&, const float angle)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (fabsf(last_tick.angular_error) > params.angular_exit_threshold) {
        if (cancel_ref) {
            last_tick.code = ExitCode::CANCELLED;
            break;
        }

        if (__timediff(start) >= params.timeout) {
            last_tick.code = ExitCode::TIMEOUT;
            break;
        }

        last_tick = turn_towards_tick(angle, params, pids);

        if (last_tick.code != ExitCode::SUCCESS) {
            break;
        }

        pros::delay(params.delay);
    }

    last_tick.time_taken_ms = __timediff(start);
    pids.reset();
    return last_tick;
}

DEFINE_STANDARD(turn_towards, controllers::PID&, const float angle)
{
    bool dummy_ref = false;
    return turn_towards_cancellable(angle, params, pids, dummy_ref);
}

DEFINE_ASYNC(turn_towards, controllers::PID&, const float angle)
{
    Future<SimpleResult> future;
    pros::Task task([future, &params, pids, angle] () mutable {
        robot::chassis.take_drive_mutex();
        future.set_value(turn_towards_cancellable(
            angle, params, pids, 
            future.get_state()->cancelled
        ));
        robot::chassis.give_drive_mutex();
    });
    return future;
}

// Useful overload for turning towards a point

DEFINE_TICK(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2f(point(0) - robot::x(), point(1) - robot::y());
    return turn_towards_tick(angle, params, pids);
}

DEFINE_CANCELLABLE(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2(point(0) - robot::x(), point(1) - robot::y());
    return turn_towards_cancellable(angle, params, pids, cancel_ref);
}

DEFINE_STANDARD(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2f(point(0) - robot::x(), point(1) - robot::y());
    return turn_towards(angle, params, pids);
}

DEFINE_ASYNC(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
{
    const float angle = atan2f(point(0) - robot::x(), point(1) - robot::y());
    return turn_towards_async(angle, params, pids);
}

DEFINE_TICK(turn_towards, controllers::PID&, const Pose& pose)
{
    const float angle = atan2(pose.x() - robot::x(), pose.y() - robot::y());
    return turn_towards_tick(angle, params, pids);
}

DEFINE_CANCELLABLE(turn_towards, controllers::PID&, const Pose& pose)
{
    const float angle = atan2(pose.x() - robot::x(), pose.y() - robot::y());
    return turn_towards_cancellable(angle, params, pids, cancel_ref);
}

DEFINE_STANDARD(turn_towards, controllers::PID&, const Pose& pose)
{
    const float angle = atan2(pose.x() - robot::x(), pose.y() - robot::y());
    return turn_towards(angle, params, pids);
}

DEFINE_ASYNC(turn_towards, controllers::PID&, const Pose& pose)
{
    const float angle = atan2(pose.x() - robot::x(), pose.y() - robot::y());
    return turn_towards_async(angle, params, pids);
}