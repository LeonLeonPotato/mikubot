#pragma once

#include "Eigen/Dense"
#include "Eigen/src/Core/Matrix.h"
#include <map>

namespace pathing {
template <int N>
class Polynomial {
    private:
        static std::map<int, Eigen::MatrixXi*> differentials;
        static Eigen::MatrixXi* build_differential_matrix(int n);

        static int falling_factorial(int i, int n);
    public:
        static Eigen::MatrixXi* get_differential(int n);
        static void clear_cache();

        Eigen::Vector<float, N> coeffs;

        Polynomial(void) { }
        Polynomial(Eigen::Vector<float, N>& coeffs) : coeffs(coeffs) { }

        void compute(const Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv = 0) const;
        Eigen::VectorXf compute(const Eigen::VectorXf& t, int deriv = 0) const;
        float compute(float t, int deriv = 0) const;

        void operator()(const Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv = 0) const { compute(t, res, deriv); }
        Eigen::VectorXf operator()(const Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        float operator()(float t, int deriv = 0) const { return compute(t, deriv); }

        std::string debug_out() const;
};

template <int N>
class Polynomial2D {
    public:
        Polynomial<N> x_poly;
        Polynomial<N> y_poly;

        Polynomial2D(void) {}
        Polynomial2D(const Polynomial<N>& x_poly, const Polynomial<N>& y_poly) : x_poly(x_poly), y_poly(y_poly) {}

        void compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv = 0) const;
        Eigen::Matrix2Xf compute(const Eigen::VectorXf& t, int deriv = 0) const;
        void compute(float t, Eigen::Vector2f& res, int deriv = 0) const;
        Eigen::Vector2f compute(float t, int deriv = 0) const;

        Eigen::Vector2f normal(float t) const;
        float angle(float t) const;
        float angular_velocity(float t) const;
        float curvature(float t) const;

        void operator()(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv = 0) const { compute(t, res, deriv); }
        Eigen::Matrix2Xf operator()(const Eigen::VectorXf& t, int deriv = 0) const { return compute(t, deriv); }
        void operator()(float t, Eigen::Vector2f& res, int deriv = 0) const { compute(t, res, deriv); }
        Eigen::Vector2f operator()(float t, int deriv = 0) const { return compute(t, deriv); }
};

template <int N>
inline std::map<int, Eigen::MatrixXi*> Polynomial<N>::differentials;

template <int N>
inline Eigen::MatrixXi* Polynomial<N>::build_differential_matrix(int n) {
    Eigen::MatrixXi* diff = new Eigen::MatrixXi(n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            (*diff)(i, j) = falling_factorial(i, j);
        }
    }
    return diff;
}

template <int N>
inline Eigen::MatrixXi* Polynomial<N>::get_differential(int n) {
    if (differentials.find(n) == differentials.end()) {
        Eigen::MatrixXi* diff = build_differential_matrix(n);
        differentials[n] = diff;
        return diff;
    }
    return differentials[n];
}

template <int N>
inline void Polynomial<N>::clear_cache() {
    for (auto& i : differentials) {
        delete i.second;
    }
    differentials.clear();
}

template <int N>
inline int Polynomial<N>::falling_factorial(int i, int n) {
    if (i < n) return 0;
    int result = 1;
    for (int j = i; j > i - n; j--) result *= j;
    return result;
}

template <int N>
inline void Polynomial<N>::compute(const Eigen::VectorXf& t, Eigen::VectorXf& res, int deriv) const {
    Eigen::VectorXf t_pow = Eigen::VectorXf::Ones(t.size());
    Eigen::MatrixXi* diff = get_differential(N);
    for (int i = deriv; i < N; i++) {
        res += t_pow * coeffs.coeffRef(i) * diff->coeffRef(i, deriv);
        t_pow.noalias() = t_pow.cwiseProduct(t);
    }
}

template <int N>
inline Eigen::VectorXf Polynomial<N>::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::VectorXf x(t.size());
    compute(t, x, deriv);
    return x;
}

template <int N>
inline float Polynomial<N>::compute(float t, int deriv) const {
    float result = 0;
    float t_pow = 1;
    Eigen::MatrixXi* diff = get_differential(N);
    for (int i = deriv; i < N; i++) {
        result += coeffs(i) * t_pow * diff->coeffRef(i, deriv);
        t_pow *= t;
    }
    return result;
}

template <int N>
inline std::string Polynomial<N>::debug_out() const {
    std::string result = "";
    for (int i = 0; i < N; i++) {
        result += std::to_string(coeffs(i)) + "t^" + std::to_string(i) + " + ";
    }
    return result.substr(0, result.size() - 3);
}

template <int N>
inline void Polynomial2D<N>::compute(const Eigen::VectorXf& t, Eigen::Matrix2Xf& res, int deriv) const {
    Eigen::VectorXf t_pow = Eigen::VectorXf::Ones(t.size());
    Eigen::MatrixXi* diff = x_poly.get_differential(N);
    for (int i = deriv; i < N; i++) {
        res.row(0) += t_pow * x_poly.coeffs.coeffRef(i) * diff->coeffRef(i, deriv);
        res.row(1) += t_pow * y_poly.coeffs.coeffRef(i) * diff->coeffRef(i, deriv);
        t_pow.noalias() = t_pow.cwiseProduct(t);
    }
}

template <int N>
inline Eigen::Matrix2Xf Polynomial2D<N>::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix2Xf x;
    compute(t, x, deriv);
    return x;
}

template <int N>
inline void Polynomial2D<N>::compute(float t, Eigen::Vector2f& res, int deriv) const {
    float t_pow = 1;
    Eigen::MatrixXi* diff = x_poly.get_differential(N);
    for (int i = deriv; i < N; i++) {
        res.coeffRef(0) += x_poly.coeffs.coeffRef(i) * t_pow * diff->coeffRef(i, deriv);
        res.coeffRef(1) += y_poly.coeffs.coeffRef(i) * t_pow * diff->coeffRef(i, deriv);
        t_pow *= t;
    }
}

template <int N>
inline Eigen::Vector2f Polynomial2D<N>::compute(float t, int deriv) const {
    return Eigen::Vector2f(x_poly(t, deriv), y_poly(t, deriv));
}

template <int N>
inline Eigen::Vector2f Polynomial2D<N>::normal(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return Eigen::Vector2f(-d(1), d(0));
}

template <int N>
inline float Polynomial2D<N>::angle(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return atan2f(d(0), d(1));
}

template <int N>
inline float Polynomial2D<N>::angular_velocity(float t) const {
    Eigen::Vector2f d1 = compute(t, 1);
    Eigen::Vector2f d2 = compute(t, 2);
    return (d1(0) * d2(1) - d1(1) * d2(0)) / (d1.dot(d1) + 1e-6);
}

template <int N>
inline float Polynomial2D<N>::curvature(float t) const {
    const Eigen::Vector2f d1 = compute(t, 1);
    const Eigen::Vector2f d2 = compute(t, 2);
    return (d1.coeffRef(0) * d2.coeffRef(1) - d1.coeffRef(1) * d2.coeffRef(0)) / (d1.dot(d1) * d1.norm() + 1e-6);
}

} // namespace pathing