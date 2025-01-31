#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple/simpletils.h"
#include "essential.h"
#include <cmath>

using namespace movement;

DEFINE_TICK(forward, controllers::PID&, const Eigen::Vector2f& line, const Eigen::Vector2f& normal) 
{
    const float reversed = 1 - 2*params.reversed;
    Eigen::Vector2f dir = Eigen::Vector2f {-normal.y(), normal.x()};
    Eigen::Vector2f closest = line + dir * (robot::pos() - line).dot(dir);
    float error_sign = 2*((line - robot::pos()).dot(normal) >= 0) - 1;
    float dist = (closest - robot::pos()).norm() * error_sign;
    float control = pids.get(dist) * reversed;
    control = std::clamp(control, -params.max_linear_speed, params.max_linear_speed);
    robot::velo(control, control);

    return {ExitCode::SUCCESS, dist, 0, 0};
}

DEFINE_CANCELLABLE(forward, controllers::PID&, const float cm)
{
    const int start = pros::millis();

    const float reversed = 1 - 2*params.reversed;
    const Eigen::Vector2f normal = Eigen::Vector2f {sinf(robot::theta()), cosf(robot::theta())} * reversed;
    const Eigen::Vector2f line = robot::pos() + cm * normal;

    SimpleResult last_tick;
    while (fabsf(last_tick.linear_error) > params.linear_exit_threshold) {
        if (cancel_ref) {
            last_tick.code = ExitCode::CANCELLED;
            break;
        }

        if (__timediff(start) >= params.timeout) {
            last_tick.code = ExitCode::TIMEOUT;
            break;
        }

        last_tick = forward_tick(line, normal, params, pids);

        if (last_tick.code != ExitCode::SUCCESS) {
            break;
        }

        pros::delay(params.delay);
    }

    last_tick.time_taken_ms = __timediff(start);
    pids.reset();
    return last_tick;
}

DEFINE_STANDARD(forward, controllers::PID&, const float cm)
{
    bool cancel = false;
    return forward_cancellable(cm, params, pids, cancel);
}

DEFINE_ASYNC(forward, controllers::PID&, const float cm)
{
    Future<SimpleResult> future;
    pros::Task task([future, &params, pids, cm] () mutable {
        future.set_value(forward_cancellable(
            cm, params, pids, 
            future.get_state()->cancelled
        ));
    });
    return future;
}