#ifndef _PROS_AUTONOMOUS_SPLINE_H_
#define _PROS_AUTONOMOUS_SPLINE_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>
#include <vector>

namespace spline {
extern Eigen::Matrix<double, 6, 6> differential_matrix_1;
extern Eigen::Matrix<double, 6, 6> differential_matrix_0;

class QuinticSpline {
    class Quintic {
        public:
            double a, b, c, d, e, f;
            double eval(double t);
            double eval_derivative(double t);
    };

    private:
        double total_length;
        std::vector<Quintic> spline_x;
        std::vector<Quintic> spline_y;

        void solve_quintic(std::vector<Quintic>& result, std::vector<double>& Y, 
            double ic_0, double ic_1, double bc_0, double bc_1);
        void solve_length(void);
        
    public:
        std::vector<Eigen::Vector2d> points;
        void solve_coeffs(double ic_theta_0, double ic_theta_1, double bc_theta_0, double bc_theta_1);
        void debug_output(void);
};
} // namespace spline

#endif