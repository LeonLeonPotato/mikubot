#pragma once

#include "autonomous/future.h"
#include "autonomous/movement/base_movement.h"

namespace movement::simple {

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

}