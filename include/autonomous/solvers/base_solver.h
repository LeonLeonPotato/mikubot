#pragma once

#include "Eigen/Dense"

namespace solvers {
typedef std::function<float(float)> func_t;
typedef std::function<Eigen::VectorXf(Eigen::VectorXf&)> func_vec_t;
} // namespace solvers