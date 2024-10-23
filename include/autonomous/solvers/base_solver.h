#pragma once

#include "Eigen/Dense"

namespace solvers {
using func_t = std::function<float(float)>;
using func_vec_t = std::function<Eigen::VectorXf(Eigen::VectorXf&)>;
} // namespace solvers