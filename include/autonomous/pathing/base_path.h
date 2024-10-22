#pragma once

#include <vector>
#include "Eigen/Dense"
#include "autonomous/solvers.h"

namespace pathing {
struct BaseParams {
    float start_heading;
    float start_magnitude;
    float end_heading;
    float end_magnitude;

    std::pair<float, float> start_cartesian() const;
    std::pair<float, float> end_cartesian() const;
};

class BasePath {
    protected:
        std::vector<float> lengths;

    public:
        std::vector<Eigen::Vector2f> points;
        
        virtual void solve_coeffs(const BaseParams& params) {}
        virtual bool need_solve() const = 0;

        virtual solvers::Solver get_solver() const { return solvers::Solver::Newton; }

        virtual void solve_lengths(int resolution = 150);
        virtual float time_parameter(const float s) const;
        virtual float arc_parameter(const float t) const;

        virtual void compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv = 0) const = 0;
        virtual Eigen::Matrix2Xf compute(const Eigen::VectorXf& t, int deriv = 0) const = 0;
        virtual void compute(float t, Eigen::Vector2f& res, int deriv = 0) const = 0;
        virtual Eigen::Vector2f compute(float t, int deriv = 0) const = 0;

        virtual Eigen::Vector2f normal(float t) const = 0;
        virtual float angle(float t) const = 0;
        virtual float angular_velocity(float t) const { return -1.0f; }
        virtual float curvature(float t) const { return -1.0f; }

        void operator()(const Eigen::VectorXf& t,Eigen::Matrix2Xf& res, int deriv = 0) const { compute(t, res, deriv); }
        Eigen::Matrix2Xf operator()(const Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        void operator()(float t, Eigen::Vector2f& res, int deriv = 0) const { compute(t, res, deriv); }
        Eigen::Vector2f operator()(float t, int deriv = 0) const { return compute(t, deriv); }
};
} // namespace pathing