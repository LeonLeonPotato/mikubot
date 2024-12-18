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

struct ProfilePoint {
    float s, t;
    float curvature;
    float left_v, center_v, right_v;
    float angular_v;
};

struct ProfileParams {
    float start_v, end_v;
    float  max_speed;
    float accel, decel;
    float track_width;
    float ds = 0.1;
    int resolution = 10000;
};

class BasePath {
    protected:
        std::vector<float> lengths;
        std::vector<ProfilePoint> profile;

    public:
        std::vector<Eigen::Vector2f> points;

        void set_relative(const Eigen::Vector2f& p);
        
        virtual void solve_coeffs(const BaseParams& params) {}
        virtual bool need_solve() const = 0;

        virtual solvers::Solver get_solver() const { return solvers::Solver::Newton; }
        virtual int maxt() const { return points.size() - 1; }

        virtual float time_parameter(const float s, int b_off = 0, int e_off = 0) const;
        virtual float arc_parameter(const float t) const;
        virtual void solve_lengths(int resolution = 5000);

        virtual void profile_path(const ProfileParams& params);
        virtual ProfilePoint profile_point(const float s) const;
        virtual const std::vector<ProfilePoint>& get_profile(void) const;

        virtual void full_sample(int resolution, Eigen::MatrixX2f& res, int deriv = 0) const;

        virtual void compute(const Eigen::ArrayXf& t, Eigen::MatrixX2f& res, int deriv = 0) const;
        virtual Eigen::MatrixX2f compute(const Eigen::ArrayXf& t, int deriv = 0) const;
        virtual void compute(float t, Eigen::Vector2f& res, int deriv = 0) const = 0;
        virtual Eigen::Vector2f compute(float t, int deriv = 0) const;

        virtual Eigen::Vector2f normal(float t) const;
        virtual float angle(float t) const;
        virtual float angular_velocity(float t) const;
        virtual float curvature(float t) const;

        template <typename... Args>
        auto operator()(Args... args) -> decltype(compute(args...)) const { return compute(args...); }

        virtual std::string debug_out() const { return ""; }
};
} // namespace pathing