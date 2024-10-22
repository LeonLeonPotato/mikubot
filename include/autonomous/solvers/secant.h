#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/solvers/base_solver.h"

namespace solvers {

std::pair<float, float> secant_single(
    func_t func,
    float t0, float t1, float start_bound, float end_bound, int iterations = 5, float threshold = 1e-1
);

std::pair<float, float> secant_vec(
    func_vec_t func,
    Eigen::VectorXf t0, Eigen::VectorXf t1, float start_bound, float end_bound, int iterations = 5, float threshold = 1e-1
);

}