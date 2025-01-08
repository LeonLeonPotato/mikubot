#pragma once

#include "ansicodes.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/pathing/polynomial.h"
#include <algorithm>
#include <cstring>
#include <iostream>

#include "Eigen/Sparse" // IWYU pragma: keep

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

        std::vector<Polynomial2D<N>> segments;

        int i_helper(float& t) const;
        float get_pascal_coefficient(int i, int j) const;
        void solve_spline(int axis, 
            const std::vector<Condition>& ics, 
            const std::vector<Condition>& bcs);
        
    public:
        static const std::vector<Condition> natural_conditions;

        NthDegreeSpline(void) { ensure(N); }
        NthDegreeSpline(int n) { segments.resize(n); ensure(N);}
        NthDegreeSpline(const std::vector<Eigen::Vector2f>& verts) { points = verts; ensure(N); }

        bool need_solve() const override { return true; }
        void solve_coeffs(const std::vector<Condition>& ics, 
            const std::vector<Condition>& bcs) override;

        void full_sample(int resolution, Eigen::MatrixX2f& res, int deriv = 0) const override;
        
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
        using BasePath::compute;

        Eigen::Vector2f normal(float t) const override;
        float angle(float t) const override;
        float angular_velocity(float t) const override;
        float curvature(float t) const override;

        std::string debug_out(void) const override;
        std::string debug_out_precise(int precision = 4) const;
};

template <int N>
inline Eigen::MatrixXf NthDegreeSpline<N>::pascal = Eigen::MatrixXf::Ones(1, 1);

template <int N>
inline std::vector<float> NthDegreeSpline<N>::zerodiffs = {1};

template <int N>
inline const std::vector<Condition> NthDegreeSpline<N>::natural_conditions = []() {
    std::vector<Condition> cs;
    for (int i = 0; i < N/2; i++) {
        cs.push_back({i+2, 0, 0});
    }
    return cs;
}();

template<int N>
void NthDegreeSpline<N>::ensure(int n) {
    if (n % 2 == 0) {
        std::cerr << PREFIX << "N (Currently " << n << ") must be odd, do not use even-degreed splines!\n";
        return;
    }

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

    // Useful constants
    const int ics_length = ics.size();
    const int bcs_length = bcs.size();
    // Originally, we have N * segments.size() unknowns, but we "collapse" the ICs into the initial rows
    int n = N * segments.size() - ics_length;
    int half = N / 2;
    int max_pivot_find = std::min(half, 3); 
    int eff_length = half + max_pivot_find + 1;

    if (ics_length != half) {
        std::cerr << PREFIX << "Invalid number of ICs: Expected " << half << ", but got " << ics_length << std::endl;
        return;
    }

    if (bcs_length != half) {
        std::cerr << PREFIX << "Invalid number of BCs: Expected " << half << ", but got " << bcs_length << std::endl;
        return;
    }

    if (ics_length > 0 && ics[0].derivative < 1) {
        std::cerr << PREFIX << "ICs must start at least at the first derivative! Current smallest derivative: %d" << ics[0].derivative << "\n";
        return;
    }

    if (bcs_length > 0 && bcs[0].derivative < 1) {
        std::cerr << PREFIX << "BCs must start at least at the first derivative! Current smallest derivative: %d" << bcs[0].derivative << "\n";
        return;
    }

    if (segments.size() == 1) { // For some reason this is a special case
        Eigen::MatrixXf A(N, N); A.setZero();
        Eigen::VectorXf B(N); B.setZero();

        for (int i = 0; i < ics_length; i++) {
            A(i, ics[i].derivative - 1) = zerodiffs[ics[i].derivative];
            B[i] = ics[i].cartesian()[axis];
        }

        A.row(half).setConstant(1.0f);
        B[half] = points[1][axis] - points[0][axis];
        for (int i = 0; i < bcs_length; i++) {
            B[N-1-i] = bcs[i].cartesian()[axis];
            // cast to float as well
            A.row(N-1-i) = Polynomial<N>::get_differential(N).col(bcs[i].derivative).tail(N).template cast<float>();
        }

        Eigen::VectorXf X = A.colPivHouseholderQr().solve(B);

        auto& poly = axis == 0 ? segments[0].x_poly : segments[0].y_poly;
        for (int i = 0; i < N; i++) {
            poly.coeffs[i+1] = X[i];
        }
        poly.coeffs[0] = points[0][axis];
        return;
    }

    // IC Collapse preparation
    bool ic_flags[N]; std::fill(ic_flags, ic_flags + N, false);
    float ic_vals[N]; std::fill(ic_vals, ic_vals + N, 0.0f);
    for (auto& ic : ics) {
        ic_flags[ic.derivative - 1] = 1;
        ic_vals[ic.derivative - 1] = ic.cartesian()[axis] / zerodiffs[ic.derivative];
    }

    // Prepare LHS N-diagonal matrix and RHS vector
    Eigen::MatrixXf A(n, N + max_pivot_find); A.setZero();
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
    A.row(n - bcs_length - 1).head(N).setConstant(1.0f);
    B[n - bcs_length - 1] = points[segments.size()][axis] - points[segments.size() - 1][axis];
    // Fill in BC differential rows
    for (int i = 0; i < bcs_length; i++) {
        int r = n - bcs_length + i;
        int d = bcs[i].derivative;
        for (int j = 0; j < N - i - 1; j++) {
            A(r, j) = Polynomial<N>::get_differential(N).coeff(j+d, d);
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
    // 12/30/2024: Why does the pascal triangle show up here?
    for (int i = N; i < n - bcs_length - 1; i++) {
        for (int j = 0; j < N; j++) {
            A(i, j) = get_pascal_coefficient(i % N, j);
        }
    }

    // (Im lazy so I just added the constants (wasted 20 bytes oh nooo!!!))
    Eigen::VectorXf X(n + eff_length - 1); X.fill(0.0f);

    bool use_own_solver = true;
    if (use_own_solver) {
        // N-diagonal banded matrix solver for Ax=B with partial pivoting (Forward pass)
        for (int i = 0; i < n - 1; i++) {
            int max_magnitude_ei = i;
            int max_magnitude_ej = half;

            // Find pivot
            for (int j = 1; j <= max_pivot_find; j++) {
                int ei = i+j; int ej = half-j;
                if (ei >= n) break;

                if (fabsf(A(ei, ej)) > fabsf(A(max_magnitude_ei, max_magnitude_ej))) {
                    max_magnitude_ei = ei;
                    max_magnitude_ej = ej;
                }
            }

            // Pivot the rows
            A.row(i).tail(eff_length).swap(A.row(max_magnitude_ei).segment(max_magnitude_ej, eff_length));
            std::swap(B[i], B[max_magnitude_ei]);

            for (int j = 1; j <= half; j++) {
                int ei = i+j; int ej = half-j;
                if (ei >= n) break;

                double alpha = A(ei, ej) / A(i, half); // Use double for precision

                // Eliminate row
                A.row(ei).segment(ej, eff_length) -= alpha * A.row(i).segment(half, eff_length);
                B[ei] -= alpha * B[i];
            }
        }

        // Backsubstitute to find values
        for (int i = n - 1; i >= 0; i--) {
            X[i] = (B[i] - A.row(i).tail(eff_length-1).dot(X.segment(i+1, eff_length-1))) / A(i, half);
        }
    } else {
        Eigen::SparseMatrix<float> sparse_A(n, n);
        for (int dst_j = 0; dst_j < n; dst_j++) {
            for (int src_i = 0; src_i < N; src_i++) {
                int dst_i = dst_j + src_i - half;
                if (dst_i < 0 || dst_i >= n) continue;
                sparse_A.insert(dst_j, dst_i) = A(dst_j, src_i);
            }
        }

        Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
        solver.compute(sparse_A);

        if (solver.info() != Eigen::Success) {
            std::cerr << PREFIX << "Failed to decompose matrix\n";
            return;
        }

        X.head(n) = solver.solve(B);
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
    return debug_out_precise(4);
}

template<int N>
std::string NthDegreeSpline<N>::debug_out_precise(int precision) const {
    std::stringstream result;

    result << "============== " << N << "-th Degree Spline ==============\n";
    for (int i = 0; i < segments.size(); i++) {
        char buf[4096];
        sprintf(buf, "P_{%d}\\left(t\\right) = \\left(%s,\\ %s\\right)\n", 
            i, 
            segments[i].x_poly.debug_out(precision).c_str(), 
            segments[i].y_poly.debug_out(precision).c_str()
        );
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

using LinearPath = NthDegreeSpline<1>;
using CubicSpline = NthDegreeSpline<3>;
using QuinticSpline = NthDegreeSpline<5>;
using SepticSpline = NthDegreeSpline<7>;
} // namespace pathing