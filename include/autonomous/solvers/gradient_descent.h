#pragma once

#include "autonomous/solvers/base_solver.h"

namespace solvers {
float gradient_descent_single(
    func_t deriv,
    float guess, float start_bound, float end_bound, 
    float step_size, int iterations = 5
);

std::pair<float, float> gradient_descent_vec(
    func_vec_t func, func_vec_t deriv,
    Eigen::VectorXf guess, float start_bound, float end_bound, 
    float step_size, int iterations = 5
);

} // namespace solvers