#include "autonomous/pathing/quintic_spline.h"
#include "ansicodes.h"

#include "Eigen/Sparse" // IWYU pragma: keep
#include "base_path.h"

#include <iostream>

using namespace pathing;

static const Eigen::Matrix<float, 5, 5> pattern {
    {1, 1, 1, 1, 1},
    {1, 2, 3, 4, -1},
    {1, 3, 6, 1, -1},
    {1, 4, -1, 1, -1},
    {1, 1, -1, 1, -1}
};

inline int QuinticSpline::i_helper(float& t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size());
    t -= i;
    return i;
}

void QuinticSpline::solve_spline(int axis, const std::vector<Condition>& ics, const std::vector<Condition>& bcs) {
    if (segments.size() == 1) return;

    // int n = 6 * segments.size();
    // std::vector<Eigen::Triplet<float>> triplets;
    // triplets.reserve(n);
    // Eigen::VectorXf B(n);

    // for (int i = 0; i < segments.size(); i++) {
    //     int r = 6 * i;
    //     triplets.emplace_back(r, r, 1);
    //     B(r) = points[i](axis);

    //     for (int k = 0; k < 6; k++) {
    //         triplets.emplace_back(r+1, k+r, differential_matrix_1.coeffRef(0, k));
    //     }
    //     B(r+1) = points[i+1](axis);

    //     if (i == segments.size() - 1) continue;

    //     for (int j = 2; j < 6; j++) {
    //         for (int k = 0; k < 6; k++) {
    //             triplets.emplace_back(r+j, k+r, differential_matrix_1.coeffRef(j-1, k));
    //             triplets.emplace_back(r+j, k+r+6, -differential_matrix_0.coeffRef(j-1, k));
    //         }
    //         B(r+j) = 0;
    //     }
    // }

    // for (int i = 0; i < 6; i++) {
    //     triplets.emplace_back(n-4, i, differential_matrix_0.coeffRef(1, i));
    //     triplets.emplace_back(n-3, i, differential_matrix_0.coeffRef(2, i));
    //     triplets.emplace_back(n-2, i+n-6, differential_matrix_1.coeffRef(1, i));
    //     triplets.emplace_back(n-1, i+n-6, differential_matrix_1.coeffRef(2, i));
    // }

    // B(n-4) = ic_0;
    // B(n-3) = ic_1;
    // B(n-2) = bc_0;
    // B(n-1) = bc_1;

    // Eigen::SparseMatrix<float> A(n, n);
    // A.setFromTriplets(triplets.begin(), triplets.end());

    // Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver;
    // solver.compute(A);

    // if (solver.info() != Eigen::Success) {
    //     std::cerr << PREFIX << "Quintic spline decomposition failed!" << std::endl;
    // }
    
    // Eigen::VectorXf X = solver.solve(B);

    // if (axis == 0) {
    //     for (int i = 0; i < segments.size(); i++) {
    //         segments[i].x_poly.coeffs = X.segment(6 * i, 6);
    //     }
    // } else {
    //     for (int i = 0; i < segments.size(); i++) {
    //         segments[i].y_poly.coeffs = X.segment(6 * i, 6);
    //     }
    // }
}

void QuinticSpline::solve_coeffs(const std::vector<Condition>& ics, const std::vector<Condition>& bcs) {
    segments.clear(); 
    segments.resize(points.size() - 1);

    solve_spline(0, ics, bcs);
    solve_spline(1, ics, bcs);
}

void QuinticSpline::full_sample(int resolution, Eigen::MatrixX2f& res, int deriv) const {
    int inc = resolution / (int) segments.size();
    const auto times = Eigen::ArrayXf::LinSpaced(inc, 0, 1);
    for (int i = 0; i < segments.size(); i++) {
        auto ref = Eigen::Ref<Eigen::MatrixX2f>(res.block(i*inc, 0, inc, 2));
        segments[i].compute(
            times,
            ref,
            deriv
        );
    }

    int remainder = resolution % segments.size();
    if (remainder != 0) {
        Eigen::Vector2f last_point = segments.back().compute(1.0f, deriv);
        res.block(resolution - remainder, 0, remainder, 2).rowwise() = last_point.transpose();
    }
}

void QuinticSpline::compute(float t, Eigen::Vector2f& res, int deriv) const {
    segments[i_helper(t)].compute(t, res, deriv);
}

Eigen::Vector2f QuinticSpline::normal(float t) const {
    return segments[i_helper(t)].normal(t);
}

float QuinticSpline::angle(float t) const {
    return segments[i_helper(t)].angle(t);
}

float QuinticSpline::angular_velocity(float t) const {
    return segments[i_helper(t)].angular_velocity(t);
}

float QuinticSpline::curvature(float t) const {
    return segments[i_helper(t)].curvature(t);
}

std::string QuinticSpline::debug_out(void) const {
    std::stringstream result;

    result << "============== Quintic Spline debug output ==============\n";
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