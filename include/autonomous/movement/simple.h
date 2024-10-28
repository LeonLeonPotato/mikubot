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
    bool& cancel_ref, int timeout = 2000, float threshold = deg(2));

MovementResult turn_towards(float angle, controllers::PID& pid, 
    int timeout = 2000, float threshold = deg(2));
MovementResult turn_towards(float angle,
    int timeout = 2000, float threshold = deg(2));

Future<MovementResult> turn_towards_async(float angle, controllers::PID& pid,
    int timeout = 2000, float threshold = deg(2));
Future<MovementResult> turn_towards_async(float angle,
    int timeout = 2000, float threshold = deg(2));

////// Go to position below

TickResult go_to_tick(const Eigen::Vector2f& point, controllers::PID& pid,
    float distance_coeff = 5.0, int max_speed = 127, int timeout = 2000, float threshold = 10);

MovementResult go_to_cancellable(const Eigen::Vector2f& point, controllers::PID& pid,
    bool& cancel_ref, float distance_coeff = 5.0, int max_speed = 127, int timeout = 2000, float threshold = 10);
MovementResult go_to_cancellable(const Eigen::Vector2f& point,
    bool& cancel_ref, float distance_coeff = 5.0, int max_speed = 127, int timeout = 2000, float threshold = 10);

MovementResult go_to(const Eigen::Vector2f& point, controllers::PID& pid,
    float distance_coeff = 5.0, int max_speed = 127, int timeout = 2000, float threshold = 10);
MovementResult go_to(const Eigen::Vector2f& point,
    float distance_coeff = 5.0, int max_speed = 127, int timeout = 2000, float threshold = 10);

Future<MovementResult> go_to_async(const Eigen::Vector2f& point, controllers::PID& pid,
    float distance_coeff = 5.0, int max_speed = 127, int timeout = 2000, float threshold = 10);
Future<MovementResult> go_to_async(const Eigen::Vector2f& point,
    float distance_coeff = 5.0, int max_speed = 127, int timeout = 2000, float threshold = 10);
} // namespace movement