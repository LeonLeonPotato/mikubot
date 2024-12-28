#pragma once

#include "autonomous/pathing/base_path.h"
#include "autonomous/pathing/polynomial.h"
#include <vector>

namespace pathing {
static const std::vector<Condition> NaturalQuinticCondition {{2, 0, 0}, {3, 0, 0}};

class QuinticSpline : public BasePath {
    private:
        std::vector<Polynomial2D<6>> segments;

        int i_helper(float& t) const;
        void solve_spline(int axis, const std::vector<Condition>& ics, const std::vector<Condition>& bcs);
        
    public:
        QuinticSpline(void) {}
        QuinticSpline(int n) { segments.resize(n); }
        QuinticSpline(const std::vector<Eigen::Vector2f>& verts) { points = verts; }

        bool need_solve() const override { return true; }
        void solve_coeffs(const std::vector<Condition>& ics, const std::vector<Condition>& bcs) override;

        void full_sample(int resolution, Eigen::MatrixX2f& res, int deriv = 0) const override;
        
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;

        Eigen::Vector2f normal(float t) const override;
        float angle(float t) const override;
        float angular_velocity(float t) const override;
        float curvature(float t) const override;

        std::string debug_out(void) const override;
};
} // namespace pathing