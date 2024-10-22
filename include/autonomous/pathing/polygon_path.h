#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/pathing/base_path.h"

namespace pathing {
class PolygonPath : public BasePath {
    public:
        PolygonPath(void) {}
        PolygonPath(const std::vector<Eigen::Vector2f>& vertices) {
            points = vertices;
        }

        bool need_solve() const override { return false; }
        solver::Solver get_solver() const override { return solver::Solver::Secant; }

        inline void compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv = 0) const override;
        inline Eigen::Matrix<float, 2, -1> compute(const Eigen::VectorXf& t, int deriv = 0) const override;
        inline void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
        inline Eigen::Vector2f compute(float t, int deriv = 0) const override;

        inline Eigen::Vector2f normal(float t) const override;
        inline float angle(float t) const override;
};

inline void PolygonPath::compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv) const {
    for (int i = 0; i < t.size(); i++) {
        res.col(i) = compute(t(i), deriv);
    }
}

inline Eigen::Matrix<float, 2, -1> PolygonPath::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix<float, 2, -1> x;
    x.resize(2, t.size());
    compute(t, x, deriv);
    return x;
}

inline void PolygonPath::compute(float t, Eigen::Vector2f& res, int deriv) const {
    int i = (int) t - (int) (t == points.size() - 1); t = t - i;
    if (deriv == 0)
        res = points[i] + (points[i + 1] - points[i]) * t;
    else if (deriv == 1)
        res = points[i + 1] - points[i];
    else
        res = Eigen::Vector2f(0, 0);
}

inline Eigen::Vector2f PolygonPath::compute(float t, int deriv) const {
    int i = (int) t - (int) (t == points.size() - 1); t = t - i;
    if (deriv == 0)
        return points[i] + (points[i + 1] - points[i]) * t;
    else if (deriv == 1)
        return points[i + 1] - points[i];
    else
        return Eigen::Vector2f(0, 0);
}

inline Eigen::Vector2f PolygonPath::normal(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return Eigen::Vector2f(-d(1), d(0));
}

inline float PolygonPath::angle(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return atan2(d(1), d(0));
}
}