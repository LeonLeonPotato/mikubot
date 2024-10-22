#pragma once

#include "autonomous/pathing/base_path.h"

namespace pathing {
class PolygonPath : public BasePath {
    public:
        PolygonPath(void) {}
        PolygonPath(const std::vector<Eigen::Vector2f>& vertices) {
            points = vertices;
        }

        bool need_solve() const override { return false; }
        solvers::Solver get_solver() const override { return solvers::Solver::Secant; }

        void compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv = 0) const override;
        Eigen::Matrix2Xf compute(const Eigen::VectorXf& t, int deriv = 0) const override;
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
        Eigen::Vector2f compute(float t, int deriv = 0) const override;

        Eigen::Vector2f normal(float t) const override;
        float angle(float t) const override;
};
} // namespace pathing