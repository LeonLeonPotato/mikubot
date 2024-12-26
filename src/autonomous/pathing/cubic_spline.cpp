#include "autonomous/pathing/cubic_spline.h"

#include "Eigen/Sparse" // IWYU pragma: keep

#include <iostream>

using namespace pathing;

const Eigen::Matrix<float, 4, 4> differential_matrix_1 {
    {1, 1, 1, 1},
    {0, 1, 2, 3},
    {0, 0, 2, 6},
    {0, 0, 0, 6}
};

const Eigen::Matrix<float, 4, 4> differential_matrix_0 {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 2, 0},
    {0, 0, 0, 6}
};

void CubicSpline::solve_spline(int axis, float ic, float bc) {
    // Solve the spline for the given axis
    // ic is the initial condition
    // bc is the boundary condition
    // axis is the axis to solve for

    int n = 4 * segments.size();

    std::vector<Eigen::Triplet<float>> triplets; triplets.reserve(n);
    Eigen::VectorXf B(n);

    triplets.emplace_back(0, 0, 1); B(0) = points[0](axis);
    triplets.emplace_back(1, 1, 1); B(1) = ic;
    triplets.emplace_back(2, n-4, 1); B(2) = bc;

    for (int i = 1; i < segments.size(); i++) {
        int r = 4 * i;
    }
}


void CubicSpline::solve_coeffs(const BaseParams& params) {
    // Solve the coefficients for the spline
    // This function is not implemented
}

void CubicSpline::compute(float t, Eigen::Vector2f& res, int deriv) const {
    // Compute the spline at the given t value
    // res is the output vector
    // deriv is the derivative to compute
    // This function is not implemented
}