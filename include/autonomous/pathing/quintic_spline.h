#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/pathing/base_path.h"
#include "autonomous/pathing/polynomial.h"

namespace pathing {
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

        void solve_coeffs(float icx0, float icx1, float icy0, float icy1,
                          float bcx0, float bcx1, float bcy0, float bcy1);
        void solve_coeffs(float icx0, float bcx0, float icy0, float bcy0);

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

inline void QuinticSpline::solve_coeffs(float icx0, float icx1, float icy0, float icy1,
                                 float bcx0, float bcx1, float bcy0, float bcy1)
{
    segments.clear(); 
    segments.resize(points.size() - 1);

    solve_spline(0, icx0, icx1, bcx0, bcx1);
    solve_spline(1, icy0, icy1, bcy0, bcy1);
}

inline void QuinticSpline::solve_coeffs(float icx0, float bcx0, float icy0, float bcy0) {
    solve_coeffs(icx0, 0, icy0, 0, bcx0, 0, bcy0, 0);
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