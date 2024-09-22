#include "autonomous/spline.h"

#include "api.h"

#include "Eigen/Sparse"
#include "Eigen/SparseCholesky"

namespace spline {
Eigen::Matrix<float, 6, 6> differential_matrix_1;
Eigen::Matrix<float, 6, 6> differential_matrix_0;

template <int N>
float Polynomial<N>::compute(float t) {
    float result = 0;
    float t_pow = 1;
    for (int i = 0; i < N; i++) {
        result += coeffs(i) * t_pow;
        t_pow *= t;
    }
    return result;
}

template <int N>
float Polynomial<N>::derivative(float t) {
    float result = 0;
    float t_pow = 1;
    for (int i = 1; i < N; i++) {
        result += coeffs(i) * t_pow * i;
        t_pow *= t;
    }
    return result;
}

template <int N>
std::string Polynomial<N>::debug_out(void) {
    std::string result = "";
    for (int i = 0; i < N; i++) {
        result += std::to_string(coeffs(i)) + "t^" + std::to_string(i) + " + ";
    }
    return result;
}

template <int N>
float Polynomial2D<N>::length(int resolution) {
    float result = 0;
    for (int i = 0; i < resolution; i++) {
        float t = (float) i / resolution;
        result += (compute(t) - compute(t + 1 / resolution)).norm();
    }
    return result;
}

void QuinticSpline::solve_spline(int axis, float ic_0, float ic_1, float bc_0, float bc_1) {
    int n = 6 * segments.size();
    std::vector<Eigen::Triplet<float>> triplets;
    triplets.reserve(n);
    Eigen::VectorXf B(n);

    for (int i = 0; i < segments.size(); i++) {
        int r = 6 * i;
        triplets.emplace_back(r, r, 1);
        B(r) = points[i](axis);

        for (int k = 0; k < 6; k++) {
            triplets.emplace_back(r+1, k+r, differential_matrix_1(0, k));
        }
        B(r+1) = points[i+1](axis);

        if (i == segments.size() - 1) continue;

        for (int j = 2; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                triplets.emplace_back(r+j, k+r, differential_matrix_1(j-1, k));
                triplets.emplace_back(r+j, k+r+6, -differential_matrix_0(j-1, k));
            }
            B(r+j) = 0;
        }
    }

    for (int i = 0; i < 6; i++) {
        triplets.emplace_back(n-4, i, differential_matrix_0(1, i));
        triplets.emplace_back(n-3, i, differential_matrix_0(2, i));
        triplets.emplace_back(n-2, i+n-6, differential_matrix_1(1, i));
        triplets.emplace_back(n-1, i+n-6, differential_matrix_1(2, i));
    }

    B(n-4) = ic_0;
    B(n-3) = ic_1;
    B(n-2) = bc_0;
    B(n-1) = bc_1;

    Eigen::SparseMatrix<float> A(n, n);
    A.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);

    if (solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed!" << std::endl;
    }
    
    Eigen::VectorXf X = solver.solve(B);

    if (axis == 0) {
        for (int i = 0; i < segments.size(); i++) {
            segments[i].x_poly.coeffs = X.segment(6 * i, 6);
        }
    } else {
        for (int i = 0; i < segments.size(); i++) {
            segments[i].y_poly.coeffs = X.segment(6 * i, 6);
        }
    }
}

void QuinticSpline::solve_coeffs(float ic_theta_0, float ic_theta_1, float bc_theta_0, float bc_theta_1) {
    segments.clear(); 
    segments.resize(points.size() - 1);

    solve_spline(0, cos(ic_theta_0), cos(ic_theta_1), cos(bc_theta_0), cos(bc_theta_1));
    solve_spline(1, sin(ic_theta_0), sin(ic_theta_1), sin(bc_theta_0), sin(bc_theta_1));
}

void QuinticSpline::solve_length(void) {
    total_length = 0;
    for (int i = 0; i < segments.size(); i++) {
        total_length += segments[i].length();
    }
}

std::string QuinticSpline::debug_out(void) {
    std::string result = "";
    for (int i = 0; i < segments.size(); i++) {
        result += "Segment " + std::to_string(i) + "\n";
        result += "X: " + segments[i].x_poly.debug_out() + "\n";
        result += "Y: " + segments[i].y_poly.debug_out() + "\n";
    }
    return result;
}

void init(void) {
    differential_matrix_1 <<
        1, 1, 1, 1, 1, 1,
        0, 1, 2, 3, 4, 5,
        0, 0, 2, 6, 12, 20,
        0, 0, 0, 6, 24, 60,
        0, 0, 0, 0, 24, 120,
        0, 0, 0, 0, 0, 120;

    differential_matrix_0 <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        0, 0, 0, 6, 0, 0,
        0, 0, 0, 0, 24, 0,
        0, 0, 0, 0, 0, 120;
}
}