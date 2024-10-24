#include "autonomous/movement.h"
#include "essential.h"

#include "api.h"

using namespace movement;

namespace movement::variables {
    float distance_coeff = 5.0;

    float turning_kP = 800;
    float turning_kI = 0.0;
    float turning_kD = 100;

    float turning_I_disable_min = -999;
    float turning_I_disable_max = 1;
    float turning_I_min = -999;
    float turning_I_max = 999;
};

std::pair<float, float> 
utils::compute_initial_t_newton(pathing::BasePath& path, solvers::func_vec_t func, solvers::func_vec_t deriv) 
{
    Eigen::VectorXf guesses = Eigen::VectorXf::LinSpaced(32, 0.05, path.points.size() - 1.05);
    return solvers::newton_vec(func, deriv, guesses, 0, path.points.size() - 1, 15);
}

std::pair<float, float> 
utils::compute_initial_t_secant(pathing::BasePath& path, solvers::func_vec_t func) {
    Eigen::VectorXf t0 = Eigen::VectorXf::LinSpaced(32, 0.05, path.points.size() - 1.1);
    Eigen::VectorXf t1 = Eigen::VectorXf::LinSpaced(32, 0.10, path.points.size() - 1.05);

    return solvers::secant_vec(func, t0, t1, 0, path.points.size() - 1, 10);
}

std::pair<float, float> 
utils::compute_updated_t_newton(pathing::BasePath& path, solvers::func_t func, solvers::func_t deriv, float t, int iterations) {
    return solvers::newton_single(func, deriv, t, t, path.points.size() - 1, iterations);
}

std::pair<float, float> 
utils::compute_updated_t_secant(pathing::BasePath& path, solvers::func_t func, float t, int iterations) {
    return solvers::secant_single(func, t, t + 1, t, path.points.size() - 1, iterations);
}

float
utils::compute_updated_t_grad_desc(pathing::BasePath& path, solvers::func_t func, float t, float step_size, int iterations) {
    return solvers::gradient_descent_single(func, t, t, path.points.size() - 1, step_size, iterations);
}

std::pair<float, float>
utils::recompute_path(pathing::BasePath& path, 
                solvers::func_vec_t func, solvers::func_vec_t deriv, 
                solvers::Solver solver,
                int goal_i,
                float end_heading, float end_magnitude,
                bool dont_solve_t)
{
    if (goal_i > 1) {
        for (int i = goal_i; i < path.points.size(); i++) {
            path.points[i - goal_i + 1] = path.points[i];
        }
        for (int i = 0; i < goal_i - 1; i++) {
            path.points.pop_back();
        }
    }

    path.points[0] = Eigen::Vector2f(robot::x, robot::y);

    if (path.need_solve()) {
        pathing::BaseParams params;
        params.start_heading = robot::theta;
        params.start_magnitude = 10;
        params.end_heading = end_heading;
        params.end_magnitude = end_magnitude;
        path.solve_coeffs(params);
    }

    if (dont_solve_t) {
        return {0, 0};
    }

    switch (solver) {
        case solvers::Solver::Newton:
            return utils::compute_initial_t_newton(path, func, deriv);
        case solvers::Solver::Secant:
            return utils::compute_initial_t_secant(path, func);
        default:
            return {-1, -1};
    }
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