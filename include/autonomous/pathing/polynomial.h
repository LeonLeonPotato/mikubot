#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "Eigen/Dense"

namespace pathing {
template <int N>
class Polynomial {
    public:
        Eigen::Vector<float, N> coeffs;

        Polynomial(void) { }
        Polynomial(Eigen::Vector<float, N>& coeffs) : coeffs(coeffs) { }

        void compute(const Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv = 0) const;
        Eigen::VectorXf compute(const Eigen::VectorXf& t, int deriv = 0) const;
        float compute(float t, int deriv = 0) const;

        inline void operator()(const Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::VectorXf operator()(const Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        inline float operator()(float t, int deriv = 0) const { return compute(t, deriv); }

        std::string debug_out() const;
};

template <int N>
class Polynomial2D {
    public:
        Polynomial<N> x_poly;
        Polynomial<N> y_poly;

        Polynomial2D(void) {}
        Polynomial2D(const Polynomial<N>& x_poly, const Polynomial<N>& y_poly) : x_poly(x_poly), y_poly(y_poly) {}

        inline void compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv = 0) const;
        inline Eigen::Matrix<float, 2, -1> compute(const Eigen::VectorXf& t, int deriv = 0) const;
        inline void compute(float t, Eigen::Vector2f& res, int deriv = 0) const;
        inline Eigen::Vector2f compute(float t, int deriv = 0) const;

        inline Eigen::Vector2f normal(float t) const;
        inline float angle(float t) const;
        inline float angular_velocity(float t) const;
        inline float curvature(float t) const;

        inline void operator()(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::Matrix<float, 2, -1> operator()(const Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        inline void operator()(float t, Eigen::Vector2f& res, int deriv = 0) const { compute(t, res, deriv); }
        inline Eigen::Vector2f operator()(float t, int deriv = 0) const { return compute(t, deriv); }
};
} // namespace pathing

template <int N>
void pathing::Polynomial<N>::compute(const Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv) const {
    Eigen::VectorXf t_pow = Eigen::VectorXf::Ones(t.size());
    for (int i = deriv; i < N; i++) {
        res += coeffs(i).cwiseProduct(t_pow) * differential_matrix_1(deriv, i);
        t_pow = t_pow.cwiseProduct(t);
    }
}

template <int N>
Eigen::VectorXf pathing::Polynomial<N>::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::VectorXf x(t.size());
    compute(t, x, deriv);
    return x;
}

template <int N>
float pathing::Polynomial<N>::compute(float t, int deriv) const {
    float result = 0;
    float t_pow = 1;
    for (int i = deriv; i < N; i++) {
        result += coeffs(i) * t_pow * differential_matrix_1(deriv, i);
        t_pow *= t;
    }
    return result;
}

template <int N>
std::string pathing::Polynomial<N>::debug_out() const {
    std::string result = "";
    for (int i = 0; i < N; i++) {
        result += std::to_string(coeffs(i)) + "t^" + std::to_string(i) + " + ";
    }
    return result;
}

template <int N>
void pathing::Polynomial2D<N>::compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv) const {
    x_poly.compute(t, res(0), deriv);
    y_poly.compute(t, res(1), deriv);
}

template <int N>
Eigen::Matrix<float, 2, -1> pathing::Polynomial2D<N>::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix<float, 2, -1> x;
    compute(t, x, deriv);
    return x;
}

template <int N>
void pathing::Polynomial2D<N>::compute(float t, Eigen::Vector2f& res, int deriv) const {
    res(0) = x_poly.compute(t, deriv);
    res(1) = y_poly.compute(t, deriv);
}

template <int N>
Eigen::Vector2f pathing::Polynomial2D<N>::compute(float t, int deriv) const {
    return Eigen::Vector2f(x_poly(t, deriv), y_poly(t, deriv));
}

template <int N>
Eigen::Vector2f pathing::Polynomial2D<N>::normal(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return Eigen::Vector2f(-d(1), d(0));
}

template <int N>
float pathing::Polynomial2D<N>::angle(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return atan2(d(1), d(0));
}

template <int N>
float pathing::Polynomial2D<N>::angular_velocity(float t) const {
    Eigen::Vector2f d1 = compute(t, 1);
    Eigen::Vector2f d2 = compute(t, 2);
    return (d1(0) * d2(1) - d1(1) * d2(0)) / d1.dot(d1);
}

template <int N>
float pathing::Polynomial2D<N>::curvature(float t) const {
    float n = compute(t, 1).norm();
    return compute(t, 2).norm() / pow(1 + n * n, 1.5);
}