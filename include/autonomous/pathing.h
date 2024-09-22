#ifndef _MIKUBOT_AUTONOMOUS_PATHING_H_
#define _MIKUBOT_AUTONOMOUS_PATHING_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/spline.h"

namespace pure_pursuit {
float compute_intersections(spline::QuinticSpline& spline, Eigen::Vector2f& point, float radius, float start_bound, float end_bound);
void follow_spline(spline::QuinticSpline& spline, float t);
} // namespace pure_pursuit

#endif