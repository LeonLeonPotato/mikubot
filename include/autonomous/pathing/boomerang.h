#pragma once

#include "autonomous/pathing/base_path.h"

namespace pathing {
class BoomerangPath : public BasePath {
    private:
        Eigen::Vector2f carrot;

    public:
        BoomerangPath(void) {}
        BoomerangPath(const std::vector<Eigen::Vector2f>& vertices) {
            points = vertices;
        }
        BoomerangPath(const Eigen::Vector2f& start, const Eigen::Vector2f& end) {
            points.push_back(start);
            points.push_back(end);
        }

        bool need_solve() const override { return true; }
        void solve_coeffs(const BaseParams& params) override;
        void solve_coeffs(float heading, float lead);
        solvers::Solver get_solver() const override { return solvers::Solver::Secant; }

        void compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv = 0) const override;
        Eigen::Matrix2Xf compute(const Eigen::VectorXf& t, int deriv = 0) const override;
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
        Eigen::Vector2f compute(float t, int deriv = 0) const override;
};
} // namespace pathing