#pragma once

#include "autonomous/pathing/base_path.h"
#include "autonomous/pathing/polynomial.h"

namespace pathing {
struct QuinticSplineParams : BaseParams {
    float start_accel_heading;
    float start_accel_magnitude;
    float end_accel_heading;
    float end_accel_magnitude;

    std::pair<float, float> start_accel_cartesian() const;
    std::pair<float, float> end_accel_cartesian() const;
};

class QuinticSpline : public BasePath {
    private:
        std::vector<Polynomial2D<6>> segments;

        void solve_spline(int axis, float ic_0, float ic_1, float bc_0, float bc_1);
        
    public:
        QuinticSpline(void) {}
        QuinticSpline(int n) { segments.resize(n); }
        QuinticSpline(const std::vector<Eigen::Vector2f>& verts) { points = verts; }

        bool need_solve() const override { return true; }
        void solve_coeffs(const QuinticSplineParams& params);
        void solve_coeffs(const BaseParams& params) override;

        void full_sample(int resolution, Eigen::MatrixX2f& res, int deriv = 0) const override;
        
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;

        Eigen::Vector2f normal(float t) const override;
        float angle(float t) const override;
        float angular_velocity(float t) const override;
        float curvature(float t) const override;

        std::string debug_out(void) const override;
};
} // namespace pathing