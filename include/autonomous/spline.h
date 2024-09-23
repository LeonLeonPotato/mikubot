#ifndef _MIKUBOT_AUTONOMOUS_SPLINE_H_
#define _MIKUBOT_AUTONOMOUS_SPLINE_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"
#include <vector>

namespace spline {
extern Eigen::Matrix<float, 6, 6> differential_matrix_1;
extern Eigen::Matrix<float, 6, 6> differential_matrix_0;

template <int N>
class Polynomial {
    public:
        Eigen::VectorXf coeffs;

        Polynomial(void) { }
        Polynomial(Eigen::VectorXf& coeffs) : coeffs(coeffs) { }

        float compute(float t);
        float derivative(float t, int n = 1);
        inline float operator()(float t) { return compute(t); }

        std::string debug_out();
};

template <int N>
class Polynomial2D {
    public:
        Polynomial<N> x_poly;
        Polynomial<N> y_poly;

        Polynomial2D(void) {}
        Polynomial2D(Polynomial<N>& x_poly, Polynomial<N>& y_poly) : x_poly(x_poly), y_poly(y_poly) {}

        inline Eigen::Vector2f compute(float t) {
            return Eigen::Vector2f(x_poly(t), y_poly(t));
        }
        inline Eigen::Vector2f derivative(float t, int n = 1) {
            return Eigen::Vector2f(x_poly.derivative(t, n), y_poly.derivative(t, n));
        }
        inline Eigen::Vector2f normal(float t) {
            Eigen::Vector2f d = derivative(t);
            return Eigen::Vector2f(-d(1), d(0));
        }
        inline float angle(float t) {
            return atan2(derivative(t)(1), derivative(t)(0));
        }
        inline float angular_velo(float t) {
            auto d1 = derivative(t, 1);
            auto d2 = derivative(t, 2);
            return d1.cross(d2) / d1.dot(d1);
        }
        inline float curvature(float t) {
            float n = derivative(t).norm();
            return derivative(t, 2).norm() / pow(1 + n * n, 1.5);
        }
        float length(int resolution = 50);

        inline Eigen::Vector2f operator()(float t) { return compute(t); }
};

class QuinticSpline {
    private:
        float total_length;
        std::vector<Polynomial2D<6>> segments;

        void solve_spline(int axis, float ic_0, float ic_1, float bc_0, float bc_1);
        
    public:
        std::vector<Eigen::Vector2f> points;
    
        QuinticSpline(void) {}
        QuinticSpline(int n) { segments.resize(n); }
        QuinticSpline(std::vector<Eigen::Vector2f>& points) : points(points) { }

        void solve_coeffs(float ic_theta_0, float ic_theta_1, float bc_theta_0, float bc_theta_1);
        void solve_length(void);
        
        inline Eigen::Vector2f compute(float t) {
            int i = (int) t;
            t = t - i + (int)(t == points.size() - 1);
            return segments[i](t);
        }
        inline Eigen::Vector2f derivative(float t, int n = 1) {
            int i = (int) t;
            t = t - i + (int)(t == points.size() - 1);
            return segments[i].derivative(t, n);
        }

        std::string debug_out(void);

        inline Eigen::Vector2f operator()(float t) { return compute(t); }
};

void init(void);
} // namespace spline

#endif