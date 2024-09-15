#ifndef _PROS_AUTONOMOUS_SPLINE_H_
#define _PROS_AUTONOMOUS_SPLINE_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>
#include <vector>

namespace spline {
extern Eigen::Matrix<float, 6, 6> differential_matrix_1;
extern Eigen::Matrix<float, 6, 6> differential_matrix_0;

template <int N>
class Polynomial {
    public:
        Eigen::Vector<float, N> coeffs;
        Polynomial(void);
        double operator()(float t);
        Polynomial<N-1> derivative(void);
};

class QuinticSpline {
    private:
        float total_length;
        std::vector<Polynomial<6>> x_polys;
        std::vector<Polynomial<6>> y_polys;

        void solve_spline(std::vector<Polynomial<6>>& result, std::vector<float>& Y, 
            float ic_0, float ic_1, float bc_0, float bc_1);
        void solve_length(std::vector<Polynomial<6>>& polys);
        
    public:
        std::vector<Eigen::Vector2d> points;
        void solve_coeffs(float ic_theta_0, float ic_theta_1, float bc_theta_0, float bc_theta_1);
        void debug_output(void);
        Eigen::Vector2d operator()(float t);
};

void init(void);
} // namespace spline

#endif