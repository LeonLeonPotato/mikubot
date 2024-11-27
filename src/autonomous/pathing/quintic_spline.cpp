#include "autonomous/pathing/quintic_spline.h"

#include "Eigen/Sparse"

#include <iostream>

using namespace pathing;

std::pair<float, float> QuinticSplineParams::start_accel_cartesian(void) const {
    return std::make_pair(
        start_accel_magnitude * sinf(start_accel_heading),
        start_accel_magnitude * cosf(start_accel_heading)
    );
}

std::pair<float, float> QuinticSplineParams::end_accel_cartesian(void) const {
    return std::make_pair(
        end_accel_magnitude * sinf(end_accel_heading),
        end_accel_magnitude * cosf(end_accel_heading)
    );
}

const Eigen::Matrix<float, 6, 6> QuinticSpline::differential_matrix_1 {
    {1, 1, 1, 1, 1, 1},
    {0, 1, 2, 3, 4, 5},
    {0, 0, 2, 6, 12, 20},
    {0, 0, 0, 6, 24, 60},
    {0, 0, 0, 0, 24, 120},
    {0, 0, 0, 0, 0, 120}
};

const Eigen::Matrix<float, 6, 6> QuinticSpline::differential_matrix_0 {
    {1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 2, 0, 0, 0},
    {0, 0, 0, 6, 0, 0},
    {0, 0, 0, 0, 24, 0},
    {0, 0, 0, 0, 0, 120}
};

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
            triplets.emplace_back(r+1, k+r, differential_matrix_1.coeffRef(0, k));
        }
        B(r+1) = points[i+1](axis);

        if (i == segments.size() - 1) continue;

        for (int j = 2; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                triplets.emplace_back(r+j, k+r, differential_matrix_1.coeffRef(j-1, k));
                triplets.emplace_back(r+j, k+r+6, -differential_matrix_0.coeffRef(j-1, k));
            }
            B(r+j) = 0;
        }
    }

    for (int i = 0; i < 6; i++) {
        triplets.emplace_back(n-4, i, differential_matrix_0.coeffRef(1, i));
        triplets.emplace_back(n-3, i, differential_matrix_0.coeffRef(2, i));
        triplets.emplace_back(n-2, i+n-6, differential_matrix_1.coeffRef(1, i));
        triplets.emplace_back(n-1, i+n-6, differential_matrix_1.coeffRef(2, i));
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

void QuinticSpline::solve_coeffs(const QuinticSplineParams& params) {
    segments.clear(); 
    segments.resize(points.size() - 1);

    auto [icx0, icy0] = params.start_cartesian();
    auto [bcx0, bcy0] = params.end_cartesian();
    auto [icx1, icy1] = params.start_accel_cartesian();
    auto [bcx1, bcy1] = params.end_accel_cartesian();
    solve_spline(0, icx0, icx1, bcx0, bcx1);
    solve_spline(1, icy0, icy1, bcy0, bcy1); 
}

void QuinticSpline::solve_coeffs(const BaseParams& params) {
    segments.clear(); 
    segments.resize(points.size() - 1);

    auto [icx0, icy0] = params.start_cartesian();
    auto [bcx0, bcy0] = params.end_cartesian();
    solve_spline(0, icx0, 0, bcx0, 0);
    solve_spline(1, icy0, 0, bcy0, 0);
}

void QuinticSpline::compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv) const {
    for (int i = 0; i < t.size(); i++) {
        res.col(i) = compute(t(i), deriv);
    }
}

Eigen::Matrix2Xf QuinticSpline::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix2Xf x;
    x.resize(2, t.size());
    compute(t, x, deriv);
    return x;
}

void QuinticSpline::compute(float t, Eigen::Vector2f& res, int deriv) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    segments[i](t, res, deriv);
}

Eigen::Vector2f QuinticSpline::compute(float t, int deriv) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i](t, deriv);
}

Eigen::Vector2f QuinticSpline::normal(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].normal(t);
}

float QuinticSpline::angle(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].angle(t);
}

float QuinticSpline::angular_velocity(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].angular_velocity(t);
}

float QuinticSpline::curvature(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].curvature(t);
}

std::string QuinticSpline::debug_out(void) const {
    std::stringstream result;
    for (int i = 0; i < segments.size(); i++) {
        char buf[4096];
        int n = sprintf(buf, "P_%d(t) = (%s, %s)\n", i, segments[i].x_poly.debug_out().c_str(), segments[i].y_poly.debug_out().c_str());
        result << buf;
    }
    return result.str();
}