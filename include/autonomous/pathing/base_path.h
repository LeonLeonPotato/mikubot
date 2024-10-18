#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include <vector>
#include <Eigen/Dense>

namespace pathing {
class BasePath {
    protected:
        std::vector<float> lengths;

    public:
        std::vector<Eigen::Vector2f> points;

        virtual void solve_lengths(int resolution = 150) = 0;
        virtual float time_parameter(float s) const = 0;
        virtual float arc_parameter(float t) const = 0;

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
}