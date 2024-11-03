#pragma once

#include "Eigen/Dense"
#include <vector>

namespace solvers {
using func_t = std::function<float(float)>;
using func_vec_t = std::function<Eigen::VectorXf(Eigen::VectorXf&)>;

struct FunctionGroup {
    std::vector<func_t> funcs;
    std::vector<func_vec_t> vec_funcs;
};
} // namespace solvers