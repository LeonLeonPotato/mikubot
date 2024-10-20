#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/controllers.h"

#include "Eigen/Dense"

namespace movement {
namespace variables {
    extern float distance_coeff;

    extern float turning_kP;
    extern float turning_kI;
    extern float turning_kD;

    extern float turning_I_disable_min;
    extern float turning_I_disable_max;
    extern float turning_I_max;
    extern float turning_I_min;
};

void init_pid(controllers::PID& pid);
void goto_pos_tick(const Eigen::Vector2f& point, controllers::PID& pid);
void turn_towards_tick(float angle, controllers::PID& pid);
void goto_pos(const Eigen::Vector2f& point, float threshold, bool correct_theta = false);
void turn_towards(float angle, float threshold);
void goto_pose(const Eigen::Vector2f& point, float angle, float threshold, float theta_threshold);
} // namespace movement