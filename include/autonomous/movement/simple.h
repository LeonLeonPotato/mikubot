#pragma once

#include "base_movement.h"

#define deg(x) (x * M_PI / 180.0)

namespace movement::simple {

////// Turn in place

TickResult&& turn_towards_tick(const float angle, controllers::PID& in_place_pid,
    const int timeout = 2000, const float threshold = deg(2));
MovementResult&& turn_towards_cancellable(const float angle, controllers::PID& pid,
    volatile bool& cancel_ref, const int timeout = 2000, const float threshold = deg(2));
MovementResult&& turn_towards(const float angle, controllers::PID& pid, 
    const int timeout = 2000, const float threshold = deg(2));
Future<MovementResult> turn_towards_async(const float angle, controllers::PID& pid,
    const int timeout = 2000, const float threshold = deg(2));

////// Swing to position

TickResult swing_to_tick(const Eigen::Vector2f& point, PIDGroup pids, 
    const bool reversed = false, const int timeout = 5000, const float max_base_speed = 1.0f, const float threshold = 5.0f);
MovementResult swing_to_cancellable(const Eigen::Vector2f& point, PIDGroup pids,
    volatile bool& cancel_ref, const bool reversed = false, const int timeout = 5000, const float max_base_speed = 1.0f, const float threshold = 5.0f);
MovementResult swing_to(const Eigen::Vector2f& point, PIDGroup pids,
    const bool reversed = false, const int timeout = 5000, const float max_base_speed = 1.0f, const float threshold = 5.0f);
Future<MovementResult> swing_to_async(const Eigen::Vector2f& point, PIDGroup pids,
    const bool reversed = false, const int timeout = 5000, const float max_base_speed = 1.0f, const float threshold = 5.0f);

////// Boomerang to position (yet to implement)

} // namespace movement