#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include <vector>
#include "Eigen/Dense"
#include "autonomous/solvers.h"

namespace pathing {
struct BaseParams {
    float start_heading;
    float start_magnitude;
    float end_heading;
    float end_magnitude;

    std::pair<float, float> start_cartesian();
    std::pair<float, float> end_cartesian();
};

class BasePath {
    protected:
        std::vector<float> lengths;

    public:
        std::vector<Eigen::Vector2f> points;
        
        virtual void solve_coeffs(BaseParams& params) {}
        virtual bool need_solve() const = 0;

        virtual solvers::Solver get_solver() const { return solvers::Solver::Newton; }

        virtual void solve_lengths(int resolution = 150);
        virtual inline float time_parameter(const float s) const;
        virtual inline float arc_parameter(const float t) const;

        virtual inline void compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv = 0) const = 0;
        virtual inline Eigen::Matrix<float, 2, -1> compute(const Eigen::VectorXf& t, int deriv = 0) const = 0;
        virtual inline void compute(float t, Eigen::Vector2f& res, int deriv = 0) const = 0;
        virtual inline Eigen::Vector2f compute(float t, int deriv = 0) const = 0;

        virtual inline Eigen::Vector2f normal(float t) const = 0;
        virtual inline float angle(float t) const = 0;
        virtual inline float angular_velocity(float t) const { return -1.0f; }
        virtual inline float curvature(float t) const { return -1.0f; }

        inline void operator()(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::Matrix<float, 2, -1> operator()(const Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        inline void operator()(float t, Eigen::Vector2f& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::Vector2f operator()(float t, int deriv = 0) const { return compute(t, deriv); }
};

inline std::pair<float, float> BaseParams::start_cartesian() {
    return std::make_pair(
        start_magnitude * sinf(start_heading), 
        start_magnitude * cosf(start_heading)
    );
}

inline std::pair<float, float> BaseParams::end_cartesian() {
    return std::make_pair(
        end_magnitude * sinf(end_heading), 
        end_magnitude * cosf(end_heading)
    );
}

inline void BasePath::solve_lengths(int resolution) {
    Eigen::VectorXf t = Eigen::VectorXf::LinSpaced(resolution, 0, points.size() - 1);
    lengths.resize(resolution + 1);
    lengths[0] = 0;
    Eigen::Matrix<float, 2, -1> res; compute(t, res);
    for (int i = 1; i < resolution + 1; i++) {
        lengths[i] = lengths[i - 1] + (res.col(i) - res.col(i - 1)).norm();
    }
}

inline float BasePath::time_parameter(const float s) const {
    int i = std::lower_bound(lengths.begin(), lengths.end(), s) - lengths.begin();
    return (float) i / lengths.size() * (points.size() - 1);
}

inline float BasePath::arc_parameter(const float t) const {
    return lengths[(int) (t / (points.size() - 1) * lengths.size())];
}
}