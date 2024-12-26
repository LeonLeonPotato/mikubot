#pragma once

#include "autonomous/future.h"
#include "autonomous/movement/base_movement.h"

namespace movement::simple {

SimpleResult boomerang_tick(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids);

SimpleResult boomerang_cancellable(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids,
    volatile bool& cancel_ref);

SimpleResult boomerang(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids);

Future<SimpleResult> boomerang_async(
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead,
    const SimpleMovementParams& params,
    PIDGroup pids);

}