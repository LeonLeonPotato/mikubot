#pragma once

#include "autonomous/pathing/base_path.h"
#include "autonomous/pathing/polynomial.h"
#include <algorithm>
#include <cstring>
#include <iostream>

#define __DEFINE_FORWARDER(name, type) \
    template <int N> \
    type NthDegreeSpline<N>::name(float t) const { \
        const int i = i_helper(t); \
        return segments[i].name(t); \
    }

namespace pathing {
template <int N>
class NthDegreeSpline : public BasePath {
    private:
        static std::vector<float> zerodiffs;
        static Eigen::MatrixXf pascal;
        static void ensure(int n);

        std::vector<Polynomial2D<N+1>> segments;

        int i_helper(float& t) const;
        float get_pascal_coefficient(int i, int j) const;
        void solve_spline(int axis, 
            const std::vector<Condition>& ics, 
            const std::vector<Condition>& bcs);
        
    public:
        NthDegreeSpline(void) { ensure(N); }
        NthDegreeSpline(int n) { segments.resize(n); ensure(N);}
        NthDegreeSpline(const std::vector<Eigen::Vector2f>& verts) { points = verts; ensure(N); }

        bool need_solve() const override { return true; }
        void solve_coeffs(const std::vector<Condition>& ics, 
            const std::vector<Condition>& bcs) override;

        void full_sample(int resolution, Eigen::MatrixX2f& res, int deriv = 0) const override;
        
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;

        Eigen::Vector2f normal(float t) const override;
        float angle(float t) const override;
        float angular_velocity(float t) const override;
        float curvature(float t) const override;

        std::string debug_out(void) const override;
};

template <int N>
Eigen::MatrixXf NthDegreeSpline<N>::pascal = Eigen::MatrixXf::Ones(1, 1);

template <int N>
std::vector<float> NthDegreeSpline<N>::zerodiffs = {1};

template<int N>
void NthDegreeSpline<N>::ensure(int n) {
    if (pascal.rows() >= n) return;

    pascal.resize(n, n);
    pascal.row(0).setConstant(1.0f);
    pascal.col(0).setConstant(1.0f);
    for (int i = 1; i < n; i++) {
        for (int j = 1; j < n - i; j++) {
            pascal(i, j) = pascal(i-1, j) + pascal(i, j-1);
        }
    }

    zerodiffs.resize(n+1);
    for (int i = 0; i < n+1; i++) {
        zerodiffs[i] = 1;
        for (int j = i; j > 0; j--) zerodiffs[i] *= j;
    }
}

template <int N>
inline int NthDegreeSpline<N>::i_helper(float& t) const {
    t = std::clamp(t, 0.0f, (float) segments.size());
    const int i = (int) t - (int) (t == segments.size());
    t -= i;
    return i;
}

template<int N>
float NthDegreeSpline<N>::get_pascal_coefficient(int i, int j) const {
    if (j >= N - i) return j % 2 == 0 ? -1 : 1;
    return NthDegreeSpline::pascal(i, j);
}

template<int N>
void NthDegreeSpline<N>::solve_spline(int axis, 
            const std::vector<Condition>& ics_og, 
            const std::vector<Condition>& bcs_og)
{
    // Sanitize and setup
    auto comp = [](const Condition& a, const Condition& b) {
        return a.derivative < b.derivative;
    };
    std::vector<Condition> ics = ics_og; std::sort(ics.begin(), ics.end(), comp);
    std::vector<Condition> bcs = bcs_og; std::sort(bcs.begin(), bcs.end(), comp);

    const int ics_length = ics.size();
    const int bcs_length = bcs.size();
    if (segments.size() == 1) return;

    // Originally, we have N * segments.size() unknowns, but IC collapse causes a reduction
    int n = N * segments.size() - ics_length;

    // IC Collapse preparation
    bool ic_flags[N]; std::fill(ic_flags, ic_flags + N, false);
    float ic_vals[N]; std::fill(ic_vals, ic_vals + N, 0.0f);
    for (auto& ic : ics) {
        ic_flags[ic.derivative - 1] = 1;
        ic_vals[ic.derivative - 1] = ic.cartesian()[axis] / zerodiffs[ic.derivative];
    }
    
    // Prepare LHS N-diagonal matrix and RHS vector
    Eigen::Matrix<float, Eigen::Dynamic, N> A(n, N); A.setZero();
    Eigen::VectorXf B(n); B.setZero();

    // Fill RHS vector with alternating sign point differences
    for (int i = 0; i < segments.size() - 1; i++) {
        for (int j = 0; j < N; j++) {
            int r = i * N + j;
            B[r] = (points[i + 1][axis] - points[i][axis])
                * ((j % 2) * -2 + 1);
        }
    }

    // Row n - bc_length - 1 for some reason needs manual filling
    A.row(n - bcs_length - 1).setConstant(1.0f);
    B[n - bcs_length - 1] = points[segments.size()][axis] - points[segments.size() - 1][axis];
    // Fill in BC differential rows
    for (int i = 0; i < bcs_length; i++) {
        int r = n - bcs_length + i;
        int d = bcs[i].derivative;
        for (int j = 0; j < N - i - 1; j++) {
            A(r, j) = Polynomial<N>::get_differential(N+1)->coeff(j+d, d);
        }
        B[r] = bcs[i].cartesian()[axis];
        if (d == 1) { // Special case for first derivative BC because it destroys the diagonal
            B[r] -= B[n - bcs_length - 1];
        }
    }

    // IC Collapse (Creates a viable N-diagonal matrix)
    for (int i = 0; i < N; i++) {
        int r = N-1-i;
        for (int j = N-1; j >= 0; j--) {
            if (i + j < N) {
                if (!ic_flags[i + j]) {
                    A(i, r--) = get_pascal_coefficient(i, j);
                } else {
                    B[i] -= ic_vals[i + j] * get_pascal_coefficient(i, j);
                }
            } else {
                A(i, j) = get_pascal_coefficient(i, j);
            }
        }
    }

    // Use Pascal triangle to fill in the rest of the matrix
    for (int i = N; i < n - bcs_length - 1; i++) {
        for (int j = 0; j < N; j++) {
            A(i, j) = get_pascal_coefficient(i % N, j);
        }
    }

    // N-diagonal "LU" decomposition algorithm (Forward pass)
    for (int i = 1; i <= N / 2; i++) {
        for (int j = 0; j < n - 1; j++) {
            if (A(j+1, i-1) != 0 && A(j, i) != 0) { // Row needs elimination
                float alpha = A(j+1, i-1) / A(j, i); // Divisor constant

                // Shift row because we are diagonal
                Eigen::Vector<float, N> row = A.row(j);
                row.head(row.size() - 1) = row.tail(row.size() - 1);
                row[row.size() - 1] = 0;

                // Eliminate row
                A.row(j+1) -= alpha * row;
                B[j+1] -= alpha * B[j];
            }
        }
    }

    // Backsubstitute to find values
    // (Im lazy so I just added N/2 (wasted 8 bytes oh nooo!!!))
    Eigen::VectorXf X(n + N/2 + 1); X.fill(0.0f);
    for (int i = n - 1; i >= 0; i--) {
        X[i] = (B[i] - A.row(i).tail(N/2).dot(X.segment(i+1, N/2))) / A(i, N/2);
    }

    // Because we collapsed ICs, we need to manually fill in the first segment
    auto& poly0 = axis == 0 ? segments[0].x_poly : segments[0].y_poly;
    poly0.coeffs[0] = points[0][axis];
    int fi = 0;
    for (int i = 0; i < N; i++) {
        if (ic_flags[i]) {
            poly0.coeffs[i+1] = ic_vals[i];
        } else {
            poly0.coeffs[i+1] = X[fi++];
        }
    }

    // Fill in the rest of the segments
    for (int i = 1; i < segments.size(); i++) {
        auto& poly = axis == 0 ? segments[i].x_poly : segments[i].y_poly;
        poly.coeffs[0] = points[i][axis];
        for (int j = 0; j < N; j++) {
            poly.coeffs[j+1] = X[fi++];
        }
    }
}

template <int N>
void NthDegreeSpline<N>::solve_coeffs(const std::vector<Condition>& ics, 
    const std::vector<Condition>& bcs) 
{
    segments.clear(); 
    segments.resize(points.size() - 1);

    solve_spline(0, ics, bcs);
    solve_spline(1, ics, bcs);
}

template <int N>
void NthDegreeSpline<N>::compute(float t, Eigen::Vector2f& res, int deriv) const {
    segments[i_helper(t)].compute(t, res, deriv);
}

template<int N>
void NthDegreeSpline<N>::full_sample(int resolution, Eigen::MatrixX2f& res, int deriv) const {
    int inc = resolution / (int) segments.size();
    int rem = resolution % segments.size();

    if (rem != 0) {
        const auto times = Eigen::ArrayXf::LinSpaced(inc + 1, 0, 1);
        for (int i = 0; i < rem; i++) {
            auto ref = Eigen::Ref<Eigen::MatrixX2f>(res.block(i*inc, 0, inc, 2));
            segments[i].compute(times, ref, deriv);
        }
    }

    const auto times = Eigen::ArrayXf::LinSpaced(inc, 0, 1);
    for (int i = rem; i < segments.size() - rem; i++) {
        auto ref = Eigen::Ref<Eigen::MatrixX2f>(res.block(i*inc, 0, inc, 2));
        segments[i].compute(times, ref, deriv);
    }
}


__DEFINE_FORWARDER(normal, Eigen::Vector2f)
__DEFINE_FORWARDER(angle, float)
__DEFINE_FORWARDER(angular_velocity, float)
__DEFINE_FORWARDER(curvature, float)

template<int N>
std::string NthDegreeSpline<N>::debug_out(void) const {
    std::stringstream result;

    result << "============== " << N << "-th Degree Spline ==============\n";
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
} // namespace pathing