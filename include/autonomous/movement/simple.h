#pragma once

#include "base_movement.h"

#define deg(x) (x * M_PI / 180.0)

namespace movement::simple {
void init_generic_pid(controllers::PID& pid);

TickResult turn_towards_tick(float angle, controllers::PID& pid,
    int timeout = 2000, float threshold = deg(2));

MovementResult turn_towards_cancellable(float angle, controllers::PID& pid,
    bool& cancel_ref, int timeout = 2000, float threshold = deg(2));
MovementResult turn_towards_cancellable(float angle,
    bool& cancel_ref, int timeout = 2000, float threshold = deg(2))
{
    controllers::PID pid; init_generic_pid(pid);
    return turn_towards_cancellable(angle, pid, cancel_ref, timeout, threshold);
}

MovementResult turn_towards(float angle, controllers::PID& pid, 
    int timeout = 2000, float threshold = deg(2))
{
    bool cancel = false;
    return turn_towards_cancellable(angle, pid, cancel, timeout, threshold);
}
MovementResult turn_towards(float angle,
    int timeout = 2000, float threshold = deg(2))
{
    bool cancel = false;
    controllers::PID pid; init_generic_pid(pid);
    return turn_towards_cancellable(angle, pid, cancel, timeout, threshold);
}

Future<MovementResult> turn_towards_async(float angle, controllers::PID& pid,
    int timeout = 2000, float threshold = deg(2))
{
    Future<MovementResult> future;
    pros::Task task([angle, &pid, timeout, threshold, &future]() {
        future.set_value(turn_towards(angle, pid, timeout, threshold));
    });
    return future;
}
Future<MovementResult> turn_towards_async(float angle,
    int timeout = 2000, float threshold = deg(2))
{
    controllers::PID pid; init_generic_pid(pid);
    Future<MovementResult> future;
    pros::Task task([angle, &pid, timeout, threshold, &future]() {
        future.set_value(turn_towards(angle, pid, timeout, threshold));
    });
    return future;
}

TickResult turn_towards_tick(const Eigen::Vector2f& point, controllers::PID& pid,
    int timeout = 2000, float threshold = deg(2))
{
    float angle = atan2(point(0), point(1));
    return turn_towards_tick(angle, pid, timeout, threshold);
}

MovementResult turn_towards_cancellable(const Eigen::Vector2f& point, controllers::PID& pid,
    bool& cancel_ref, int timeout = 2000, float threshold = deg(2))
{
    float angle = atan2(point(0), point(1));
    return turn_towards_cancellable(angle, pid, cancel_ref, timeout, threshold);
}
MovementResult turn_towards_cancellable(const Eigen::Vector2f& point,
    bool& cancel_ref, int timeout = 2000, float threshold = deg(2))
{
    float angle = atan2(point(0), point(1));
    controllers::PID pid; init_generic_pid(pid);
    return turn_towards_cancellable(angle, pid, cancel_ref, timeout, threshold);
}

MovementResult turn_towards(const Eigen::Vector2f& point, controllers::PID& pid, 
    int timeout = 2000, float threshold = deg(2))
{
    float angle = atan2(point(0), point(1));
    bool cancel = false;
    return turn_towards_cancellable(angle, pid, cancel, timeout, threshold);
}
MovementResult turn_towards(const Eigen::Vector2f& point,
    int timeout = 2000, float threshold = deg(2))
{
    float angle = atan2(point(0), point(1));
    bool cancel = false;
    controllers::PID pid; init_generic_pid(pid);
    return turn_towards_cancellable(angle, pid, cancel, timeout, threshold);
}

Future<MovementResult> turn_towards_async(const Eigen::Vector2f& point, controllers::PID& pid,
    int timeout = 2000, float threshold = deg(2))
{
    float angle = atan2(point(0), point(1));
    Future<MovementResult> future;
    pros::Task task([angle, &pid, timeout, threshold, &future]() {
        future.set_value(turn_towards(angle, pid, timeout, threshold));
    });
    return future;
}
Future<MovementResult> turn_towards_async(const Eigen::Vector2f& point,
    int timeout = 2000, float threshold = deg(2))
{
    float angle = atan2(point(0), point(1));
    controllers::PID pid; init_generic_pid(pid);
    Future<MovementResult> future;
    pros::Task task([angle, &pid, timeout, threshold, &future]() {
        future.set_value(turn_towards(angle, pid, timeout, threshold));
    });
    return future;
}
} // namespace movement