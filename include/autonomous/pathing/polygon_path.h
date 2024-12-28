#pragma once

#include "autonomous/pathing/base_path.h"

namespace pathing {
class PolygonPath : public BasePath {
    public:
        bool need_solve() const override { return false; }
        solvers::Solver get_solver() const override { return solvers::Solver::Secant; }

        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
};
} // namespace pathing