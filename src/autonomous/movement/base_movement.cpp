#include "autonomous/movement.h"
#include "essential.h"

#include "api.h"

using namespace movement;

namespace movement::variables {
    float distance_coeff = 5.0;

    float turning_kP = 300;
    float turning_kI = 1;
    float turning_kD = 0;

    float turning_I_disable_min = -999;
    float turning_I_disable_max = 1;
    float turning_I_min = -999;
    float turning_I_max = 999;
};

std::pair<float, float> 
utils::compute_initial_t_newton(pathing::BasePath& path, solvers::func_vec_t func, solvers::func_vec_t deriv) 
{
    Eigen::VectorXf guesses = Eigen::VectorXf(32);
    guesses.LinSpaced(32, 0.05, path.points.size() - 1.05);
    return solvers::newton_vec(func, deriv, guesses, 0, path.points.size() - 1, 15);
}

std::pair<float, float> 
utils::compute_initial_t_secant(pathing::BasePath& path, solvers::func_vec_t func) {
    Eigen::VectorXf guesses = Eigen::VectorXf(32);
    guesses.LinSpaced(32, 0.05, path.points.size() - 1.10);

    Eigen::VectorXf guesses2 = Eigen::VectorXf(32);
    guesses2.LinSpaced(32, 0.10, path.points.size() - 1.05);

    return solvers::secant_vec(func, guesses, guesses2, 0, path.points.size() - 1, 10);
}

inline std::pair<float, float> 
utils::compute_updated_t_newton(pathing::BasePath& path, solvers::func_t func, solvers::func_t deriv, float t, int iterations) {
    return solvers::newton_single(func, deriv, t, t, path.points.size() - 1, iterations);
}

inline std::pair<float, float> 
utils::compute_updated_t_secant(pathing::BasePath& path, solvers::func_t func, float t, int iterations) {
    return solvers::secant_single(func, t, t + 0.05, t, path.points.size() - 1, iterations);
}

void movement::init_pid(controllers::PID& pid) {
    pid.kp = variables::turning_kP;
    pid.ki = variables::turning_kI;
    pid.kd = variables::turning_kD;
    pid.integral_min = variables::turning_I_min;
    pid.integral_max = variables::turning_I_max;
    pid.disable_integral_lower = variables::turning_I_disable_min;
    pid.disable_integral_upper = variables::turning_I_disable_max;
}

void movement::goto_pos_tick(const Eigen::Vector2f& point, controllers::PID& pid) {
    float theta_diff = robot::angular_diff(point);
    float dist = robot::distance(point);
    float dist_vel = fmin(dist * variables::distance_coeff, 127);
    pid.register_error(fabs(theta_diff));
    float turn_vel = pid.get();
    robot::volt(
        dist_vel + theta_diff * turn_vel, 
        dist_vel - theta_diff * turn_vel
    );
}

void movement::turn_towards_tick(float angle, controllers::PID& pid) {
    float theta_diff = robot::angular_diff(angle);
    pid.register_error(fabs(theta_diff));
    float turn_vel = pid.get();
    robot::volt(
        theta_diff * turn_vel, 
        -theta_diff * turn_vel
    );
}

void movement::goto_pos(const Eigen::Vector2f& point, float threshold) {
    controllers::PID pid;
    init_pid(pid);
    
    while (robot::distance(point) > threshold) {
        goto_pos_tick(point, pid);
        pros::delay(20);
    }
}

void movement::turn_towards(float angle, float threshold) {
    controllers::PID pid;
    init_pid(pid);

    while (fabs(robot::angular_diff(angle)) > threshold) {
        turn_towards_tick(angle, pid);
        pros::delay(20);
    }
}

void movement::goto_pose(const Eigen::Vector2f& point, float angle, float threshold, float theta_threshold) {
    controllers::PID pid;
    init_pid(pid);

    while (robot::distance(point) > threshold) {
        goto_pos_tick(point, pid);
        pros::delay(20);
    }
    pid.reset();
    printf("Angle: %f\n", robot::angular_diff(angle));
    while (fabs(robot::angular_diff(angle)) > theta_threshold) {
        turn_towards_tick(angle, pid);
        printf("Angle: %f\n", robot::angular_diff(angle));
        pros::delay(20);
    }
    robot::brake();
}