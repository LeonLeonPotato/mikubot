#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"

namespace movement {
namespace pure_pursuit {
std::pair<float, float> compute_intersections(pathing::BasePath& path, const Eigen::Vector2f& point, float radius, 
                            float guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1);
std::pair<float, float> compute_intersections(pathing::BasePath& path, const Eigen::Vector2f& point, float radius, 
                            Eigen::VectorXf guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1);

float follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, float radius, int iterations = 5);
void follow_path(pathing::BasePath& path, float radius, int iterations = 5);
} // namespace pure_pursuit
} // namespace movement