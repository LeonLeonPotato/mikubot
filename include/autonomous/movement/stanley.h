#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"

namespace movement {
namespace stanley {

// std::pair<float, float> secant_closest(pathing::BasePath& path, const Eigen::Vector2f& point,
//                             float t0, float t1, float start_bound, float end_bound, int iterations, float threshold) 
// {
//     float ft1;
//     while (iterations--) {
//         Eigen::Vector2f pt1 = path.compute(t1) - point;
//         ft1 = pt1.dot(pt1);

//         if (ft1 < threshold) break;

//         Eigen::Vector2f pt0 = path.compute(t0) - point;
//         float ft0 = pt0.dot(pt0);

//         float t2 = t1 - ft1 * (t1 - t0) / (ft1 - ft0);
//         t2 = std::clamp(t2, start_bound, end_bound);
//         t0 = t1; t1 = t2;
//     }

//     return {t1, ft1};
// }

}
}