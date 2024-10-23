#include "autonomous/pathing/boomerang.h"

using namespace pathing;

void BoomerangPath::solve_coeffs(const BaseParams& params) {
    carrot = points.back() - Eigen::Vector2f(
        params.end_magnitude * sinf(params.end_heading), 
        params.end_magnitude * cosf(params.end_heading)
    );
}

#include <iostream>
void BoomerangPath::solve_coeffs(float heading, float lead) {
    float dist = (points.back() - points.front()).norm();
    carrot = points.back() - Eigen::Vector2f(
        dist * sinf(heading) * lead, 
        dist * cosf(heading) * lead
    );
}

void BoomerangPath::compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv) const {
    for (int i = 0; i < t.size(); i++) {
        res.col(i) = compute(t(i), deriv);
    }
}

Eigen::Matrix2Xf BoomerangPath::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix2Xf x;
    x.resize(2, t.size());
    compute(t, x, deriv);
    return x;
}

void BoomerangPath::compute(float t, Eigen::Vector2f& res, int deriv) const {
    const Eigen::Vector2f& start = points.front();
    const Eigen::Vector2f& end = points.back();
    switch (deriv) {
        case 0:
            res = start + 2*t*(carrot - start) + t*t*(start - 2*carrot + end);
            break;
        case 1:
            res = 2*(carrot - start) + 2*t*(start - 2*carrot + end);
            break;
        case 2:
            res = 2*(start - 2*carrot + end);
            break;
        default:
            res = Eigen::Vector2f(0, 0);
            break;
    }
}

Eigen::Vector2f BoomerangPath::compute(float t, int deriv) const {
    Eigen::Vector2f res;
    compute(t, res, deriv);
    return res;
}