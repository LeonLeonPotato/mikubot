#include "autonomous/movement/simple/boomerang.h"
#include "ansicodes.h"
#include "essential.h"
#include "pros/rtos.h"

using namespace movement;

DEFINE_TICK(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
{
    const float true_target_dist = robot::distance(point);
    const Eigen::Vector2f carrot = point
         - lead * true_target_dist * Eigen::Vector2f(sinf(angle), cosf(angle));
    const float angle_diff = robot::angular_diff(carrot, params.reversed);

    // printf("%sCarrot: [%f, %f]\n", PREFIX.c_str(), carrot.x(), carrot.y());

    float speed = pids.linear.get(robot::distance(carrot));
    float turn = pids.angular.get(angle_diff);
    if (params.reversed) speed = -speed;
    if (params.use_cosine_scaling) speed *= cosf(angle_diff);
    speed = std::clamp(speed, -params.max_linear_speed, params.max_linear_speed);

    // printf("%s%f, %f\n", PREFIX.c_str(), speed, turn);
    robot::velo(speed + turn, speed - turn);

    return { ExitCode::SUCCESS, true_target_dist, 0 };
}

DEFINE_CANCELLABLE(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
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

        pros::c::task_delay(20);
    }

    return { ExitCode::SUCCESS, last_tick.error, __timediff(start) };
}

DEFINE_STANDARD(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
{
    bool cancel = false;
    return boomerang_cancellable(point, angle, lead, params, pids, cancel);
}

DEFINE_ASYNC(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
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

