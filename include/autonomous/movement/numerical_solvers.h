#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/pathing/base_path.h"

namespace movement {
namespace solvers {

typedef std::function<float(float)> func_t;
typedef std::function<float(float)> deriv_t;
typedef std::function<Eigen::VectorXf(Eigen::VectorXf&)> func_vec_t;
typedef std::function<Eigen::VectorXf(Eigen::VectorXf&)> deriv_vec_t;

std::pair<float, float> newton(
    func_t func, deriv_t deriv,
    float guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1
);

std::pair<float, float> newton(
    func_vec_t func, deriv_vec_t deriv,
    Eigen::VectorXf guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1
);

std::pair<float, float> secant(
    func_t func,
    float t0, float t1, float start_bound, float end_bound, int iterations = 5, float threshold = 1e-1
);

std::pair<float, float> secant(
    func_vec_t func,
    Eigen::VectorXf t0, Eigen::VectorXf t1, float start_bound, float end_bound, int iterations = 5, float threshold = 1e-1
);

} // namespace solvers
} // namespace movement