#include "autonomous/pathing/cubic_spline.h"
#include "Eigen/src/Core/Matrix.h"
#include "ansicodes.h"

#include "Eigen/Sparse" // IWYU pragma: keep

#include <iostream>

using namespace pathing;

static const Eigen::Matrix<float, 3, 3> pattern {
    {1, 1, 1},
    {1, 2, -1},
    {1, 1, -1}
};

inline int CubicSpline::i_helper(float& t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size());
    t -= i;
    return i;
}

static void tridiangular(Eigen::MatrixX3f& A, Eigen::VectorXf& B, Eigen::VectorXf& X) {
    const int n = A.rows();
    float alpha = A(1, 0) / A(0, 1);

    // Eliminate all ai's
    for (int i = 1; i < n; i++) {
        A(i, 0) -= alpha * A(i-1, 1);
        A(i, 1) -= alpha * A(i-1, 2);
        B(i) -= alpha * B(i-1);
        alpha = A(i+(i != n-1), 0) / A(i, 1);
    }

    // Last row should be x_n * b_n = d_n
    X(n-1) = B(n-1) / A(n-1, 1);
    for (int i = n - 2; i >= 0; i--) {
        X(i) = (B(i) - A(i, 2) * X(i+1)) / A(i, 1);
    }
}

void CubicSpline::solve_spline(int axis, const std::vector<Condition>& ics, const std::vector<Condition>& bcs) {
    const float ic = ics[0].cartesian()[axis];
    const float bc = bcs[0].cartesian()[axis];

    if (segments.size() == 1) {
        auto& poly = axis == 0 ? segments[0].x_poly : segments[0].y_poly;
        poly.coeffs = {
            points[0][axis],
            ic,
            3 * (points[1][axis] - points[0][axis]) - 2 * ic - bc,
            2 * (points[0][axis] - points[1][axis]) + ic + bc
        };
        return;
    }

    const int n = 3 * segments.size() - 1;

    Eigen::MatrixX3f A(n, 3);
    Eigen::VectorXf B(n);

    // Initial rows
    A.block(0, 0, 3, 3) = Eigen::Matrix3f {
        {0, 1, 1},
        {2, 3, -1},
        {3, 1, -2}
    };
    B[0] = points[1](axis) - points[0](axis) - ic;
    B[1] = -ic;
    B[2] = ic;

    // Inner point method (the meat of the algorithm)
    for (int i = 1; i < segments.size() - 1; i++) {
        const int r = 3 * i;
        for (int j = 0; j < 3; j++) {
            A.row(r + j) = pattern.row(j);
            B[r + j] = (points[i + 1](axis) - points[i](axis)) * ((j % 2) * -2 + 1);
        }
    }

    const float lastdiff = points[segments.size()](axis)
        - points[segments.size() - 1](axis);

    // Row n-2
    A.row(n-2).setConstant(1.0f);
    B[n-2] = lastdiff;
    
    // Row n-1
    A.row(n-1) = Eigen::Vector3f {1, 2, 0};
    B[n-1] = bc - lastdiff;

    // Solve the system
    Eigen::VectorXf X(n);
    tridiangular(A, B, X);

    // Manually do the first segment as it uses IC instead of X
    auto& poly0 = axis == 0 ? segments[0].x_poly : segments[0].y_poly;
    poly0.coeffs = {points[0](axis), ic, X[0], X[1]};

    for (int i = 1; i < segments.size(); i++) {
        auto& poly = axis == 0 ? segments[i].x_poly : segments[i].y_poly;
        int r = 3*i-1; // Skip a2 and a3
        poly.coeffs = {points[i](axis), X[r], X[r+1], X[r+2]};
    }
}


void CubicSpline::solve_coeffs(const std::vector<Condition>& ics, const std::vector<Condition>& bcs) {
    segments.clear(); 
    segments.resize(points.size() - 1);

    solve_spline(0, ics, bcs);
    solve_spline(1, ics, bcs);
}

void CubicSpline::compute(float t, Eigen::Vector2f& res, int deriv) const {
    segments[i_helper(t)].compute(t, res, deriv);
}

Eigen::Vector2f CubicSpline::normal(float t) const {
    return segments[i_helper(t)].normal(t);
}

float CubicSpline::angle(float t) const {
    return segments[i_helper(t)].angle(t);
}

float CubicSpline::angular_velocity(float t) const {
    return segments[i_helper(t)].angular_velocity(t);
}

float CubicSpline::curvature(float t) const {
    return segments[i_helper(t)].curvature(t);
}

void CubicSpline::full_sample(int resolution, Eigen::MatrixX2f& res, int deriv) const {
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