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
    // printf("Speed: %f, Turn: %f, Angular: %f, Reversed: %d, Dist: %f, Pos: [%f, %f]\n", speed, turn, angle_diff, reversed, dist, robot::pos().x(), robot::pos().y());
    robot::velo(speed + turn, speed - turn);

    return { ExitCode::SUCCESS, dist, angle_diff, 0 };
}

DEFINE_CANCELLABLE(swing_to, PIDGroup, const Eigen::Vector2f& point)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (last_tick.linear_error > params.linear_exit_threshold) {
        if (cancel_ref) {
            last_tick.code = ExitCode::CANCELLED;
            break;
        }

        if (__timediff(start) >= params.timeout) {
            last_tick.code = ExitCode::TIMEOUT;
            break;
        }

        last_tick = swing_to_tick(point, params, pids);

        if (last_tick.code != ExitCode::SUCCESS) {
            break;
        }

        pros::delay(params.delay);
    }

    last_tick.time_taken_ms = __timediff(start);
    pids.reset();
    return last_tick;
}

DEFINE_STANDARD(swing_to, PIDGroup, const Eigen::Vector2f& point)
{
    bool cancel = false;
    return swing_to_cancellable(point, params, pids, cancel);
}

DEFINE_ASYNC(swing_to, PIDGroup, const Eigen::Vector2f& point)
{
    Future<SimpleResult> future;
    pros::Task task([future, &params, pids, &point] () mutable {
        future.set_value(swing_to_cancellable(
            point, params, pids, 
            future.get_state()->cancelled
        ));
    });
    return future;
}