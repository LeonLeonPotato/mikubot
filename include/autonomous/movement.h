#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/pathing.h"
#include "autonomous/controllers.h"

namespace movement {
namespace variables {
    extern float distance_coeff;

    extern controllers::PID turning_pid;
};

namespace pure_pursuit {
std::pair<float, float> compute_intersections(pathing::BasePath& spline, const Eigen::Vector2f& point, float radius, 
                            float guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1);
std::pair<float, float> compute_intersections(pathing::BasePath& spline, const Eigen::Vector2f& point, float radius, 
                            Eigen::VectorXf guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1);
void follow_spline(pathing::BasePath& spline, float t);
} // namespace pure_pursuit
} // namespace movement