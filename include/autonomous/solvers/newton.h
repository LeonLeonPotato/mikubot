#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/solvers/base_solver.h"

namespace solvers {

std::pair<float, float> newton_single(
    func_t func, func_t deriv,
    float guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1
);

std::pair<float, float> newton_vec(
    func_vec_t func, func_vec_t deriv,
    Eigen::VectorXf guess, float start_bound, float end_bound, int iterations = 5, float threshold=1e-1
);

}