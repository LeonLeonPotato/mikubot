#pragma once

#include "autonomous/future.h"
#include "autonomous/movement/base_movement.h"

namespace movement::simple {

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

// Useful overload for turning towards a point

SimpleResult turn_towards_tick(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& in_place_pid);

SimpleResult turn_towards_cancellable(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid,
    volatile bool& cancel_ref);

SimpleResult turn_towards(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid);

Future<SimpleResult> turn_towards_async(
    const Eigen::Vector2f& point, 
    const SimpleMovementParams& params,
    controllers::PID& pid);

}