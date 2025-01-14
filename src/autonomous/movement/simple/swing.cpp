#include "autonomous/movement/simple/swing.h"
#include "Eigen/src/Core/Matrix.h"
#include "autonomous/movement/base_movement.h"
#include "essential.h"
#include "simpletils.h"

using namespace movement;

DEFINE_TICK(swing_to, PIDGroup, const Eigen::Vector2f& point)
{
    const float dist = robot::distance(point);
    const float angle_diff = robot::angular_diff(point, params.reversed);
    float speed = pids.linear.get(dist);
    float turn = pids.angular.get(angle_diff);
    if (params.reversed) speed = -speed;
    if (params.use_cosine_scaling) speed *= cosf(angle_diff);
    speed = std::clamp(speed, -params.max_linear_speed, params.max_linear_speed);
    // printf("Speed: %f, Turn: %f, Angular: %f, Reversed: %d, Dist: %f, Pos: [%f, %f]\n", speed, turn, angle_diff, reversed, dist, robot::pos.x(), robot::pos.y());
    robot::velo(speed + turn, speed - turn);

    return { ExitCode::SUCCESS, dist, 0 };
}

DEFINE_CANCELLABLE(swing_to, PIDGroup, const Eigen::Vector2f& point)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (robot::distance(point) > params.linear_exit_threshold && 
           fabs(robot::angular_diff(point, params.reversed)) > params.angular_exit_threshold)
    {
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

DEFINE_STANDARD(swing_to, PIDGroup, const Eigen::Vector2f& point)
{
    bool cancel = false;
    return swing_to_cancellable(point, params, pids, cancel);
}

DEFINE_ASYNC(swing_to, PIDGroup, const Eigen::Vector2f& point)
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