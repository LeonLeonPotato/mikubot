#pragma once

#include "base_movement.h"

#define deg(x) (x * M_PI / 180.0)

namespace movement::simple {

////// Turn in place

SimpleResult turn_towards_tick(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid);

SimpleResult turn_towards_cancellable(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& pid,
    volatile bool& cancel_ref);

SimpleResult turn_towards(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& pid);

Future<SimpleResult> turn_towards_async(
    const float angle, 
    const SimpleMovementParams& params,
    controllers::PID& pid);

////// Face point

SimpleResult face_tick(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid);

SimpleResult face_cancellable(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid,
    volatile bool& cancel_ref);

SimpleResult face(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid);

Future<SimpleResult> face_async(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid);

////// Swing to position

SimpleResult swing_to_tick(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    PIDGroup pids);

SimpleResult swing_to_cancellable(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    PIDGroup pids,
    volatile bool& cancel_ref);

SimpleResult swing_to(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    PIDGroup pids);

Future<SimpleResult> swing_to_async(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    PIDGroup pids);

////// Boomerang to position (yet to implement)

} // namespace movement