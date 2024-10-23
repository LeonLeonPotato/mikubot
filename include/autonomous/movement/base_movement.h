#pragma once

#include "autonomous/controllers.h"
#include "autonomous/pathing/base_path.h"

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
} // namespace variables

namespace utils {
std::pair<float, float> 
compute_initial_t_newton(pathing::BasePath& path, solvers::func_vec_t func, solvers::func_vec_t deriv);

std::pair<float, float> 
compute_initial_t_secant(pathing::BasePath& path, solvers::func_vec_t func);

std::pair<float, float> 
compute_updated_t_newton(pathing::BasePath& path, solvers::func_t func, solvers::func_t deriv, float t, int iterations);

std::pair<float, float> 
compute_updated_t_secant(pathing::BasePath& path, solvers::func_t func, float t, int iterations);

float
compute_updated_t_grad_desc(pathing::BasePath& path, solvers::func_t func, float t, float step_size, int iterations);

std::pair<float, float>
recompute_path(pathing::BasePath& path, 
                solvers::func_vec_t func, solvers::func_vec_t deriv, 
                solvers::Solver solver,
                int goal_i, bool dont_solve_t = false);
} // namespace utils

void init_pid(controllers::PID& pid);
void goto_pos_tick(const Eigen::Vector2f& point, controllers::PID& pid);
void turn_towards_tick(float angle, controllers::PID& pid);
void goto_pos(const Eigen::Vector2f& point, float threshold);
void turn_towards(float angle, float threshold);
void goto_pose(const Eigen::Vector2f& point, float angle, float threshold, float theta_threshold);
} // namespace movement