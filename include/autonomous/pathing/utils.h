#pragma once

#include "Eigen/Dense"

namespace pathing::utils {
const int falling_factorial(int i, int j);
const Eigen::MatrixXi& get_differential_matrix(int n);
const Eigen::MatrixXi& get_pascal_matrix(int n);

void destroy_differential_matrix(void);
void destroy_pascal_matrix(void);
} // namespace pathing::utils