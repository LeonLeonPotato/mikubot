#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "Eigen/Dense"

namespace movement {
namespace solvers {

typedef std::function<float(float)> func_t;
typedef std::function<Eigen::VectorXf(Eigen::VectorXf&)> func_vec_t;

std::pair<float, float> newton(
    func_t func, func_t deriv,
    float guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1
);

std::pair<float, float> newton(
    func_vec_t func, func_t deriv,
    Eigen::VectorXf guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1
);

}
}