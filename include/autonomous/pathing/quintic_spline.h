#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/pathing/base_path.h"
#include "autonomous/pathing/polynomial.h"

namespace pathing {
struct QuinticSplineParams : BaseParams {
    float start_accel_heading;
    float start_accel_magnitude;
    float end_accel_heading;
    float end_accel_magnitude;

    std::pair<float, float> start_accel_cartesian();
    std::pair<float, float> end_accel_cartesian();
};

class QuinticSpline : public BasePath {
    private:
        static const Eigen::Matrix<float, 6, 6> differential_matrix_1;
        static const Eigen::Matrix<float, 6, 6> differential_matrix_0;

        std::vector<Polynomial2D<6>> segments;

        void solve_spline(int axis, float ic_0, float ic_1, float bc_0, float bc_1);
        
    public:
        QuinticSpline(void) {}
        QuinticSpline(int n) { segments.resize(n); }
        QuinticSpline(const std::vector<Eigen::Vector2f>& verts) { points = verts; }

        bool need_solve() const override { return true; }
        void solve_coeffs(QuinticSplineParams& params);
        void solve_coeffs(BaseParams& params) override;

        void compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv = 0) const override;
        Eigen::Matrix<float, 2, -1> compute(const Eigen::VectorXf& t, int deriv = 0) const override;
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
        Eigen::Vector2f compute(float t, int deriv = 0) const override;

        Eigen::Vector2f normal(float t) const override;
        float angle(float t) const override;
        float angular_velocity(float t) const override;
        float curvature(float t) const override;

        std::string debug_out(void) const;
};

inline std::pair<float, float> QuinticSplineParams::start_accel_cartesian(void) {
    return std::make_pair(
        start_accel_magnitude * sinf(start_accel_heading),
        start_accel_magnitude * cosf(start_accel_heading)
    );
}

inline std::pair<float, float> QuinticSplineParams::end_accel_cartesian(void) {
    return std::make_pair(
        end_accel_magnitude * sinf(end_accel_heading),
        end_accel_magnitude * cosf(end_accel_heading)
    );
}

inline const Eigen::Matrix<float, 6, 6> QuinticSpline::differential_matrix_1 {
    {1, 1, 1, 1, 1, 1},
    {0, 1, 2, 3, 4, 5},
    {0, 0, 2, 6, 12, 20},
    {0, 0, 0, 6, 24, 60},
    {0, 0, 0, 0, 24, 120},
    {0, 0, 0, 0, 0, 120}
};

inline const Eigen::Matrix<float, 6, 6> QuinticSpline::differential_matrix_0 {
    {1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 2, 0, 0, 0},
    {0, 0, 0, 6, 0, 0},
    {0, 0, 0, 0, 24, 0},
    {0, 0, 0, 0, 0, 120}
};

inline void QuinticSpline::solve_coeffs(QuinticSplineParams& params)
{
    segments.clear(); 
    segments.resize(points.size() - 1);

    auto [icx0, icy0] = params.start_cartesian();
    auto [bcx0, bcy0] = params.end_cartesian();
    auto [icx1, icy1] = params.start_accel_cartesian();
    auto [bcx1, bcy1] = params.end_accel_cartesian();
    solve_spline(0, icx0, icx1, bcx0, bcx1);
    solve_spline(1, icy0, icy1, bcy0, bcy1);
}

inline void QuinticSpline::solve_coeffs(BaseParams& params) {
    segments.clear(); 
    segments.resize(points.size() - 1);

    auto [icx0, icy0] = params.start_cartesian();
    auto [bcx0, bcy0] = params.end_cartesian();
    solve_spline(0, icx0, 0, bcx0, 0);
    solve_spline(1, icy0, 0, bcy0, 0);
}

inline void QuinticSpline::compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv) const {
    for (int i = 0; i < t.size(); i++) {
        res.col(i) = compute(t(i), deriv);
    }
}

inline Eigen::Matrix<float, 2, -1> QuinticSpline::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix<float, 2, -1> x;
    x.resize(2, t.size());
    compute(t, x, deriv);
    return x;
}

inline void QuinticSpline::compute(float t, Eigen::Vector2f& res, int deriv) const {
    int i = (int) t - (int) (t == segments.size()); t = t - i;
    segments[i].compute(t, res, deriv);
}

inline Eigen::Vector2f QuinticSpline::compute(float t, int deriv) const {
    int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].compute(t, deriv);
}

inline Eigen::Vector2f QuinticSpline::normal(float t) const {
    int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].normal(t);
}

inline float QuinticSpline::angle(float t) const {
    int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].angle(t);
}

inline float QuinticSpline::angular_velocity(float t) const {
    int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].angular_velocity(t);
}

inline float QuinticSpline::curvature(float t) const {
    int i = (int) t - (int) (t == segments.size()); t = t - i;
    return segments[i].curvature(t);
}
}