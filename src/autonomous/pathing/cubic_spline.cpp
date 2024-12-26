#include "autonomous/pathing/cubic_spline.h"
#include "ansicodes.h"

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

    for (int i = 0; i < segments.size() - 1; i++) {
        int r = 4 * i + 2;
        for (int k = 0; k < 3; k++) {
            for (int j = 0; j < 3; j++) { // Derivative continuity loop
                triplets.emplace_back(r + k, r + j + k - 1, differential_matrix_1.coeffRef(k+1, j+1));
                triplets.emplace_back(r + k, r + j + k + 3, -differential_matrix_0.coeffRef(k+1, j+1));
                B[r + k] = 0;
            }
        }
        // C0 continuity
        triplets.emplace_back(r + 4, r + 4, 1);
        B[r + 4] = points[i+1](axis);
    }

    for (int i = 0; i < 4; i++) {
        triplets.emplace_back(n-2, n-1-i, 1);
    }
    triplets.emplace_back(n-1, n-2, 2);
    triplets.emplace_back(n-1, n-1, 6);
    B[n-2] = ic;
    B[n-1] = points[segments.size()](axis);

    Eigen::SparseMatrix<float> A(n, n);
    A.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);

    if (solver.info() != Eigen::Success) {
        std::cerr << PREFIX << "Cubic spline decomposition failed!" << std::endl;
    }
    
    Eigen::VectorXf X = solver.solve(B);

    if (axis == 0) {
        for (int i = 0; i < segments.size(); i++) {
            segments[i].x_poly.coeffs = X.segment(4 * i, 4);
        }
    } else {
        for (int i = 0; i < segments.size(); i++) {
            segments[i].y_poly.coeffs = X.segment(4 * i, 4);
        }
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