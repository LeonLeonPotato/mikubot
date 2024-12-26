#include "autonomous/movement/simple/swing.h"
#include "essential.h"

using namespace movement;

SimpleResult simple::swing_to_tick(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    PIDGroup pids)
{
    const float dist = robot::distance(point);
    const float angle_diff = robot::angular_diff(point, params.reversed);
    float speed = pids.linear.get(dist);
    float turn = pids.angular.get(angle_diff);
    if (params.reversed) speed = -speed;
    if (params.use_cosine_scaling) speed *= fmax(0, cosf(angle_diff));
    speed = std::clamp(speed, -params.max_linear_speed, params.max_linear_speed);
    // printf("Speed: %f, Turn: %f, Angular: %f, Reversed: %d, Dist: %f, Pos: [%f, %f]\n", speed, turn, angle_diff, reversed, dist, robot::pos.x(), robot::pos.y());
    robot::velo(speed + turn, speed - turn);

    return { ExitCode::SUCCESS, dist, 0 };
}

SimpleResult simple::swing_to_cancellable(
    const Eigen::Vector2f& point, 
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

        last_tick = swing_to_tick(point, params, pids);
        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, __timediff(start) };
        }

        pros::delay(params.delay);
    }

    return { ExitCode::SUCCESS, last_tick.error, __timediff(start) };
}

SimpleResult simple::swing_to(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    PIDGroup pids)
{
    bool cancel = false;
    return swing_to_cancellable(point, params, pids, cancel);
}

Future<SimpleResult> simple::swing_to_async(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    PIDGroup pids)
{
    Future<SimpleResult> future;
    pros::Task task([&point, &params, &pids, &future] () {
        future.set_value(swing_to_cancellable(
            point, params, pids, 
            future.get_state()->cancelled
        ));
    });
    return future;
}