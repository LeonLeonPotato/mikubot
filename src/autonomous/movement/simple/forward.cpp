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
    Eigen::Vector2f closest = line + dir * (robot::pos - line).dot(dir);
    float dist = (closest - robot::pos).norm();
    float control = pids.get(dist) * reversed;
    control = std::clamp(control, -params.max_linear_speed, params.max_linear_speed);
    robot::velo(control, control);

    float error_sign = 2*((line - robot::pos).dot(normal) >= 0) - 1;
    return {.code = ExitCode::SUCCESS, .error = dist * error_sign, .time_taken_ms = 0};
}

DEFINE_CANCELLABLE(forward, controllers::PID&, const float cm)
{
    const int start = pros::millis();
    const float reversed = 1 - 2*params.reversed;
    const Eigen::Vector2f normal = Eigen::Vector2f {sinf(robot::theta), cosf(robot::theta)} * reversed;
    const Eigen::Vector2f line = robot::pos + cm * normal;

    SimpleResult last_tick {.error = infinityf()};
    while (last_tick.error > params.exit_threshold) {
        if (cancel_ref) {
            return { ExitCode::CANCELLED, last_tick.error, __timediff(start) };
        }

        if (pros::millis() - start > params.timeout) {
            return { ExitCode::TIMEOUT, last_tick.error, __timediff(start) };
        }

        last_tick = forward_tick(line, normal, params, pids);
        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, __timediff(start) };
        }

        pros::delay(params.delay);
    }

    return { ExitCode::SUCCESS, last_tick.error, __timediff(start) };
}

DEFINE_STANDARD(forward, controllers::PID&, const float cm)
{
    bool cancel = false;
    return forward_cancellable(cm, params, pids, cancel);
}

DEFINE_ASYNC(forward, controllers::PID&, const float cm)
{
    Future<SimpleResult> future;
    pros::Task task([&] () {
        future.set_value(forward_cancellable(
            cm, params, pids, 
            future.get_state()->cancelled
        ));
    });
    return future;
}