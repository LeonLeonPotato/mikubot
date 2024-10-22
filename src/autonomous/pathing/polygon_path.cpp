#include "autonomous/pathing/polygon_path.h"

using namespace pathing;

void PolygonPath::compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv) const {
    for (int i = 0; i < t.size(); i++) {
        res.col(i) = compute(t(i), deriv);
    }
}

Eigen::Matrix2Xf PolygonPath::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix2Xf x;
    x.resize(2, t.size());
    compute(t, x, deriv);
    return x;
}

void PolygonPath::compute(float t, Eigen::Vector2f& res, int deriv) const {
    int i = (int) t - (int) (t == points.size() - 1); t = t - i;
    if (deriv == 0)
        res = points[i] + (points[i + 1] - points[i]) * t;
    else if (deriv == 1)
        res = points[i + 1] - points[i];
    else
        res = Eigen::Vector2f(0, 0);
}

Eigen::Vector2f PolygonPath::compute(float t, int deriv) const {
    int i = (int) t - (int) (t == points.size() - 1); t = t - i;
    if (deriv == 0)
        return points[i] + (points[i + 1] - points[i]) * t;
    else if (deriv == 1)
        return points[i + 1] - points[i];
    else
        return Eigen::Vector2f(0, 0);
}

Eigen::Vector2f PolygonPath::normal(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return Eigen::Vector2f(-d(1), d(0));
}

float PolygonPath::angle(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return atan2(d(1), d(0));
}