#ifndef _MIKUBOT_AUTONOMOUS_SPLINE_H_
#define _MIKUBOT_AUTONOMOUS_SPLINE_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"
#include <vector>
#include <iostream>

#define LOCALIZE_T int i = (int) t - (int) (t == segments.size()); t = t - i;
typedef Eigen::Matrix<float, 2, -1> Matrix2Nf;

namespace spline {
extern Eigen::Matrix<float, 6, 6> differential_matrix_1;
extern Eigen::Matrix<float, 6, 6> differential_matrix_0;

template <int N>
class Polynomial {
    public:
        Eigen::Vector<float, N> coeffs;

        Polynomial(void) { }
        Polynomial(Eigen::Vector<float, N>& coeffs) : coeffs(coeffs) { }

        void compute(Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv = 0) const;
        Eigen::VectorXf compute(Eigen::VectorXf& t, int deriv = 0) const {
            Eigen::VectorXf x(t.size());
            compute(t, x, deriv);
            return x;
        }
        float compute(float t, int deriv = 0) const;

        inline void operator()(Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::VectorXf operator()(Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        inline float operator()(float t, int deriv = 0) const { return compute(t, deriv); }

        std::string debug_out() const;
};

template <int N>
class Polynomial2D {
    public:
        Polynomial<N> x_poly;
        Polynomial<N> y_poly;

        Polynomial2D(void) {}
        Polynomial2D(Polynomial<N>& x_poly, Polynomial<N>& y_poly) : x_poly(x_poly), y_poly(y_poly) {}

        inline void compute(Eigen::VectorXf& t, Matrix2Nf& res, int deriv = 0) const {
            x_poly.compute(t, res(0), deriv);
            y_poly.compute(t, res(1), deriv);
        }
        inline Matrix2Nf compute(Eigen::VectorXf& t, int deriv = 0) const {
            Matrix2Nf x;
            compute(t, x, deriv);
            return x;
        }
        inline void compute(float t, Eigen::Vector2f& res, int deriv = 0) const {
            res(0) = x_poly.compute(t, deriv);
            res(1) = y_poly.compute(t, deriv);
        }
        inline Eigen::Vector2f compute(float t, int deriv = 0) const {
            return Eigen::Vector2f(x_poly(t, deriv), y_poly(t, deriv));
        }

        inline Eigen::Vector2f normal(float t) const {
            Eigen::Vector2f d = compute(t, 1);
            return Eigen::Vector2f(-d(1), d(0));
        }
        inline float angle(float t) const {
            Eigen::Vector2f d = compute(t, 1);
            return atan2(d(1), d(0));
        }
        inline float angular_velocity(float t) const {
            Eigen::Vector2f d1 = compute(t, 1);
            Eigen::Vector2f d2 = compute(t, 2);
            return (d1(0) * d2(1) - d1(1) * d2(0)) / d1.dot(d1);
        }
        inline float curvature(float t) const {
            float n = compute(t, 1).norm();
            return compute(t, 2).norm() / pow(1 + n * n, 1.5);
        }

        inline void operator()(Eigen::VectorXf& t, Matrix2Nf& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Matrix2Nf operator()(Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        inline void operator()(float t, Eigen::Vector2f& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::Vector2f operator()(float t, int deriv = 0) const { return compute(t, deriv); }
};

class AbstractSpline {
    protected:
        std::vector<float> lengths;

    public:
        std::vector<Eigen::Vector2f> points;

        virtual void solve_lengths(int resolution = 150) = 0;
        virtual float time_parameter(float s) const = 0;
        virtual float arc_parameter(float t) const = 0;

        virtual inline void compute(Eigen::VectorXf& t, Matrix2Nf& res, int deriv = 0) const = 0;
        virtual inline Matrix2Nf compute(Eigen::VectorXf& t, int deriv = 0) const = 0;
        virtual inline void compute(float t, Eigen::Vector2f& res, int deriv = 0) const = 0;
        virtual inline Eigen::Vector2f compute(float t, int deriv = 0) const = 0;

        virtual inline Eigen::Vector2f normal(float t) const = 0;
        virtual inline float angle(float t) const = 0;
        virtual inline float angular_velocity(float t) const = 0;
        virtual inline float curvature(float t) const = 0;

        inline void operator()(Eigen::VectorXf& t, Matrix2Nf& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Matrix2Nf operator()(Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        inline void operator()(float t, Eigen::Vector2f& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::Vector2f operator()(float t, int deriv = 0) const { return compute(t, deriv); }
};

class QuinticSpline : public AbstractSpline {
    private:
        std::vector<Polynomial2D<6>> segments;

        void solve_spline(int axis, float ic_0, float ic_1, float bc_0, float bc_1);
        
    public:
        std::vector<Eigen::Vector2f> points;
    
        QuinticSpline(void) {}
        QuinticSpline(int n) { segments.resize(n); }
        QuinticSpline(std::vector<Eigen::Vector2f>& points) : points(points) { }

        void solve_lengths(int resolution = 150) override;
        inline float time_parameter(float s) const override {
            int i = std::lower_bound(lengths.begin(), lengths.end(), s) - lengths.begin();
            return (float) i / lengths.size() * segments.size();
        }
        inline float arc_parameter(float t) const override {
            return lengths[(int) (t * lengths.size())];
        }

        inline void solve_coeffs(float icx0, float icx1, float icy0, float icy1,
                          float bcx0, float bcx1, float bcy0, float bcy1) 
        {
            segments.clear(); 
            segments.resize(points.size() - 1);

            solve_spline(0, icx0, icx1, bcx0, bcx1);
            solve_spline(1, icy0, icy1, bcy0, bcy1);
        }

        inline void compute(Eigen::VectorXf& t, Matrix2Nf& res, int deriv = 0) const override {
            for (int j = 0; j < t.size(); j++) {
                int i = (int) t(j) - (int) (t(j) == points.size() - 1);
                float tj = t(j) - i;
                res.col(j) = segments[i].compute(tj, deriv);
            }
        }
        inline Matrix2Nf compute(Eigen::VectorXf& t, int deriv = 0) const override {
            Matrix2Nf x;
            x.resize(2, t.size());
            compute(t, x, deriv);
            return x;
        }
        inline void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override {
            LOCALIZE_T
            segments[i].compute(t, res, deriv);
        }
        inline Eigen::Vector2f compute(float t, int deriv = 0) const override {
            LOCALIZE_T
            return segments[i].compute(t, deriv);
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
};

void init(void);
} // namespace spline

#endif