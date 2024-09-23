#ifndef _MIKUBOT_AUTONOMOUS_SPLINE_H_
#define _MIKUBOT_AUTONOMOUS_SPLINE_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"
#include <vector>

#define LOCALIZE_T int i = (int) t; t = t - i + (int)(t == points.size() - 1);

namespace spline {
extern Eigen::Matrix<float, 6, 6> differential_matrix_1;
extern Eigen::Matrix<float, 6, 6> differential_matrix_0;

template <int N>
class Polynomial {
    public:
        Eigen::VectorXf coeffs;

        Polynomial(void) { }
        Polynomial(Eigen::VectorXf& coeffs) : coeffs(coeffs) { }

        float compute(float t) const;
        float derivative(float t, int n = 1) const;
        inline float operator()(float t) const { return compute(t); }

        std::string debug_out() const;
};

template <int N>
class Polynomial2D {
    public:
        Polynomial<N> x_poly;
        Polynomial<N> y_poly;

        Polynomial2D(void) {}
        Polynomial2D(Polynomial<N>& x_poly, Polynomial<N>& y_poly) : x_poly(x_poly), y_poly(y_poly) {}

        inline Eigen::Vector2f compute(float t) const {
            return Eigen::Vector2f(x_poly(t), y_poly(t));
        }
        inline Eigen::Vector2f derivative(float t, int n = 1) const {
            return Eigen::Vector2f(x_poly.derivative(t, n), y_poly.derivative(t, n));
        }
        inline Eigen::Vector2f normal(float t) const {
            Eigen::Vector2f d = derivative(t);
            return Eigen::Vector2f(-d(1), d(0));
        }
        inline float angle(float t) const {
            return atan2(derivative(t)(1), derivative(t)(0));
        }
        inline float angular_velocity(float t) const {
            auto d1 = derivative(t, 1);
            auto d2 = derivative(t, 2);
            // return d1.cross(d2) / d1.dot(d1);
            return 0;
        }
        inline float curvature(float t) const {
            float n = derivative(t).norm();
            return derivative(t, 2).norm() / pow(1 + n * n, 1.5);
        }

        inline Eigen::Vector2f operator()(float t) const { return compute(t); }
};

class AbstractSpline {
    protected:
        std::vector<float> lengths;

    public:
        std::vector<Eigen::Vector2f> points;

        virtual void solve_lengths(int resolution = 50) = 0;
        virtual float time_parameter(float s) const = 0;
        virtual float arc_parameter(float t) const = 0;

        virtual inline Eigen::Vector2f compute(float t) const = 0;
        virtual inline Eigen::Vector2f derivative(float t, int n = 1) const = 0;
        virtual inline Eigen::Vector2f normal(float t) const = 0;
        virtual inline float angle(float t) const = 0;
        virtual inline float angular_velocity(float t) const = 0;
        virtual inline float curvature(float t) const = 0;
};

class QuinticSpline : AbstractSpline {
    private:
        std::vector<Polynomial2D<6>> segments;

        void solve_spline(int axis, float ic_0, float ic_1, float bc_0, float bc_1);
        
    public:
        std::vector<Eigen::Vector2f> points;
    
        QuinticSpline(void) {}
        QuinticSpline(int n) { segments.resize(n); }
        QuinticSpline(std::vector<Eigen::Vector2f>& points) : points(points) { }

        inline void solve_coeffs(float icx0, float icx1, float icy0, float icy1,
                          float bcx0, float bcx1, float bcy0, float bcy1) 
        {
            segments.clear(); 
            segments.resize(points.size() - 1);

            solve_spline(0, icx0, icx1, bcx0, bcx1);
            solve_spline(1, icy0, icy1, bcy0, bcy1);
        }

        void solve_length(int resolution = 50);

        inline float time_parameter(float s) const override {
            int i = std::lower_bound(lengths.begin(), lengths.end(), s) - lengths.begin();
            return (float) i / lengths.size() * segments.size();
        }
        inline float arc_parameter(float t) const override {
            return lengths[(int) (t * lengths.size())];
        }
        
        inline Eigen::Vector2f compute(float t) const override {
            LOCALIZE_T
            return segments[i].compute(t);
        }
        inline Eigen::Vector2f derivative(float t, int n = 1) const override {
            LOCALIZE_T
            return segments[i].derivative(t, n);
        }
        inline Eigen::Vector2f normal(float t) const override {
            LOCALIZE_T
            return segments[i].normal(t);
        }
        inline float angle(float t) const override {
            LOCALIZE_T
            return segments[i].angle(t);
        }
        inline float angular_velocity(float t) const override {
            LOCALIZE_T
            return segments[i].angular_velocity(t);
        }
        inline float curvature(float t) const override {
            LOCALIZE_T
            return segments[i].curvature(t);
        }

        std::string debug_out(void) const;

        inline Eigen::Vector2f operator()(float t) const { return compute(t); }
};

void init(void);
} // namespace spline

#endif