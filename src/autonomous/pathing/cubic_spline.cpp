#include "autonomous/pathing/cubic_spline.h"
#include "ansicodes.h"

#include "Eigen/Sparse" // IWYU pragma: keep

#include <iostream>

using namespace pathing;

static const Eigen::Matrix<float, 4, 4> differential_matrix_1 {
    {1, 1, 1, 1},
    {0, 1, 2, 3},
    {0, 0, 2, 6},
    {0, 0, 0, 6}
};

static const Eigen::Matrix<float, 4, 4> differential_matrix_0 {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 2, 0},
    {0, 0, 0, 6}
};

void CubicSpline::solve_spline(int axis, float ic, float bc) {
    int n = 4 * segments.size();

    std::vector<Eigen::Triplet<float>> triplets; triplets.reserve(n);
    Eigen::VectorXf B(n);

    triplets.emplace_back(0, 0, 1); B(0) = points[0](axis);
    triplets.emplace_back(1, 1, 1); B(1) = ic;

    for (int i = 0; i < segments.size() - 1; i++) {
        int r = 4 * i;
        for (int k = 0; k < 3; k++) {
            for (int j = k; j < 4; j++) {
                triplets.emplace_back(r + k + 2, r + j, differential_matrix_1(k, j));
            }
        }
        // C0 continuity part 1 (fn(1) = p[n+1])
        B[r + 2] = points[i+1](axis);

        // C1 continuity
        triplets.emplace_back(r + 3, r + 5, -1);
        B[r + 3] = 0;

        // C2 continuity
        triplets.emplace_back(r + 4, r + 6, -2);
        B[r + 4] = 0;

        // C0 continuity part 2 (fn+1(0) = p[n+1])
        triplets.emplace_back(r + 5, r + 4, 1);
        B[r + 5] = points[i+1](axis);
    }

    for (int i = 0; i < 4; i++) {
        triplets.emplace_back(n-2, n-1-i, 1);
        triplets.emplace_back(n-1, n-4+i, i);
    }
    B[n-2] = points[segments.size()](axis);
    B[n-1] = bc;

    Eigen::SparseMatrix<float> A(n, n);
    A.setFromTriplets(triplets.begin(), triplets.end());

    std::cout << A << std::endl;
    std::cout << B << std::endl;

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
    segments.clear(); 
    segments.resize(points.size() - 1);

    auto [icx, icy] = params.start_cartesian();
    auto [bcx, bcy] = params.end_cartesian();
    solve_spline(0, icx, bcx);
    solve_spline(1, icy, bcy);
}

void CubicSpline::compute(float t, Eigen::Vector2f& res, int deriv) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    segments[i].compute(t, res, deriv);
}

Eigen::Vector2f CubicSpline::normal(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].normal(t);
}

float CubicSpline::angle(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].angle(t);
}

float CubicSpline::angular_velocity(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].angular_velocity(t);
}

float CubicSpline::curvature(float t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].curvature(t);
}

std::string CubicSpline::debug_out(void) const {
    std::stringstream result;

    result << "============== Cubic Spline debug output ==============\n";
    for (int i = 0; i < segments.size(); i++) {
        char buf[4096];
        sprintf(buf, "P_{%d}\\left(t\\right) = \\left(%s,\\ %s\\right)\n", i, segments[i].x_poly.debug_out().c_str(), segments[i].y_poly.debug_out().c_str());
        result << buf;
    }

    result << "P\\left(t\\right) = \\left\\{";
    for (int i = 0; i < segments.size(); i++) {
        result << i << "\\le t";
        if (i != segments.size()-1) result << " < ";
        else result << "\\le ";
        result << (i + 1) << ": P_{" << i << "}\\left(t - " << i << "\\right)";
        if (i != segments.size()-1) result << ",\\ ";
    }
    result << "\\right\\}\n";

    result << "N = \\left[";
    for (int i = 0; i < points.size(); i++) {
        const auto& p = points[i];
        result << "\\left(" << p.x() << ",\\ " << p.y() << "\\right)";
        if (i != points.size()-1) result << ",\\ ";
    }
    result << "\\right]\n";
    result << "=========================================================";

    return result.str();
}