#include "autonomous/movement/simple/boomerang.h"
#include "essential.h"

using namespace movement;

SimpleResult simple::boomerang_tick(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids) 
{
    const float true_target_dist = robot::distance(point);
    const Eigen::Vector2f carrot = point
         - lead * true_target_dist * Eigen::Vector2f(sinf(angle), cosf(angle));
    const float angle_diff = robot::angular_diff(carrot, params.reversed);

    float speed = pids.linear.get(true_target_dist);
    float turn = pids.angular.get(angle_diff);
    if (params.reversed) speed = -speed;
    if (params.use_cosine_scaling) speed *= fmax(0, cosf(angle_diff));
    speed = std::clamp(speed, -params.max_linear_speed, params.max_linear_speed);
    robot::velo(speed + turn, speed - turn);

    return { ExitCode::SUCCESS, true_target_dist, 0 };
}

SimpleResult simple::boomerang_cancellable(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids,
    volatile bool& cancel_ref)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (robot::distance(point) > params.exit_threshold) {
        if (cancel_ref) {
            return { ExitCode::CANCELLED, last_tick.error, __timediff(start) };
        }

        if (pros::millis() - start > params.timeout) {
            return { ExitCode::TIMEOUT, last_tick.error, __timediff(start) };
        }

        last_tick = boomerang_tick(point, angle, lead, params, pids);
        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, __timediff(start) };
        }

        pros::delay(params.delay);
    }

    return { ExitCode::SUCCESS, last_tick.error, __timediff(start) };
}

SimpleResult simple::boomerang(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids)
{
    bool cancel = false;
    return boomerang_cancellable(point, angle, lead, params, pids, cancel);
}

Future<SimpleResult> simple::boomerang_async(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids)
{
    Future<SimpleResult> future;
    pros::Task task([&point, angle, lead, params, pids, &future] () {
        future.set_value(boomerang_cancellable(
            point, angle, lead, params, pids, 
            future.get_state()->cancelled
        ));
    });
    return future;
}

