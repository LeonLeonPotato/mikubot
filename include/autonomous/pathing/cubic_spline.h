#pragma once

#include "autonomous/pathing/base_path.h"
#include "autonomous/pathing/polynomial.h"

namespace pathing {
using CubicSplineParams = BaseParams;

class CubicSpline : public BasePath {
    private:
        static const Eigen::Matrix<float, 4, 4> differential_matrix_1;
        static const Eigen::Matrix<float, 4, 4> differential_matrix_0;

        std::vector<Polynomial2D<4>> segments;

        void solve_spline(int axis, float ic, float bc);
        
    public:
        CubicSpline(void) {}
        CubicSpline(int n) { segments.resize(n); }
        CubicSpline(const std::vector<Eigen::Vector2f>& verts) { points = verts; }

        bool need_solve() const override { return true; }
        void solve_coeffs(const BaseParams& params) override;

        void compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv = 0) const override;
        Eigen::Matrix2Xf compute(const Eigen::VectorXf& t, int deriv = 0) const override;
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
        Eigen::Vector2f compute(float t, int deriv = 0) const override;

        Eigen::Vector2f normal(float t) const override;
        float angle(float t) const override;
        float angular_velocity(float t) const override;
        float curvature(float t) const override;

        std::string debug_out(void) const override;
};
} // namespace pathing