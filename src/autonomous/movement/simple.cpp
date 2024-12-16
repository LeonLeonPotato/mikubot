#include "autonomous/movement/simple.h"
#include "essential.h"

using namespace movement;

////// Turn in place

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
            return { ExitCode::CANCELLED, last_tick.error, (int) (pros::millis() - start) };
        }

        if (pros::millis() - start >= params.timeout) {
            return { ExitCode::TIMEOUT, last_tick.error, (int) (pros::millis() - start) };
        }

        last_tick = turn_towards_tick(angle, params, pid);

        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, (int) (pros::millis() - start) };
        }

        pros::delay(params.delay);
    }

    return { ExitCode::SUCCESS, last_tick.error, (int) (pros::millis() - start) };
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

////// Face point

SimpleResult simple::face_tick(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid)
{
    const float angle = atan2(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_tick(angle, params, in_place_pid);
}

SimpleResult simple::face_cancellable(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid,
    volatile bool& cancel_ref)
{
    const float angle = atan2(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_cancellable(angle, params, pid, cancel_ref);
}

SimpleResult simple::face(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid)
{
    const float angle = atan2f(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards(angle, params, in_place_pid);
}

Future<SimpleResult> simple::face_async(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid)
{
    const float angle = atan2(point(0) - robot::pos.x(), point(1) - robot::pos.y());
    return turn_towards_async(angle, params, pid);
}

////// Swing to position

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
            return { ExitCode::CANCELLED, last_tick.error, (int) (pros::millis() - start) };
        }

        if (pros::millis() - start > params.timeout) {
            return { ExitCode::TIMEOUT, last_tick.error, (int) (pros::millis() - start) };
        }

        last_tick = swing_to_tick(point, params, pids);
        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, (int) (pros::millis() - start) };
        }

        pros::delay(params.delay);
    }

    return { ExitCode::SUCCESS, last_tick.error, (int) (pros::millis() - start) };
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