#ifndef _MIKUBOT_AUTONOMOUS_PATHING_H_
#define _MIKUBOT_AUTONOMOUS_PATHING_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/spline.h"

namespace pure_pursuit {
float compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, float radius, 
                            float guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1);
float compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, float radius, 
                            Eigen::VectorXf guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1);
void follow_spline(spline::AbstractSpline& spline, float t);
} // namespace pure_pursuit

#endif