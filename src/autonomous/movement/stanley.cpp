#include "autonomous/movement/stanley.h"
#include "autonomous/movement/base_movement.h"
#include "essential.h"

using namespace movement;

namespace movement::stanley::variables {
    float kP = 0.1;
    float kI = 0.01;
    float kD = 0.01;

    float I_disable_min = -999;
    float I_disable_max = 10;
    float I_max = 999;
    float I_min = -999;
};

void stanley_init_pid(controllers::PID& pid) {
    pid.kp = stanley::variables::kP;
    pid.ki = stanley::variables::kI;
    pid.kd = stanley::variables::kD;
    pid.disable_integral_lower = stanley::variables::I_disable_min;
    pid.disable_integral_upper = stanley::variables::I_disable_max;
    pid.integral_min = stanley::variables::I_min;
    pid.integral_max = stanley::variables::I_max;
}

float stanley::follow_path_tick(pathing::BasePath& path, 
                                        controllers::PID& turn_pid, controllers::PID& track_pid, 
                                        solvers::func_t deriv, float t,
                                        int iterations)
{
    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::Vector2f& last_point = path.points.back();

    t = utils::compute_updated_t_grad_desc(path, deriv, t, 0.0001, iterations);

    Eigen::Vector2f res = path.compute(t);
    Eigen::Vector2f tangent = path.compute(t, 1);
    float theta = atan2(tangent(1), tangent(0));

    float dtheta = robot::angular_diff(theta);
    turn_pid.register_error(fabs(dtheta));

    float track_error = robot::distance(res);
    float dtheta_track = 0;
    if (track_error != 0) {
        dtheta_track = robot::angular_diff(res);
    }
    track_pid.register_error(track_error);

    float speed = fminf(robot::distance(last_point) * movement::variables::distance_coeff, 127);
    float turn_amount = turn_pid.get();
    float track_amount = track_pid.get();

    robot::volt(
        (int) (speed + turn_amount * dtheta + track_amount * dtheta_track),
        (int) (speed - turn_amount * dtheta - track_amount * dtheta_track)
    );
    return t;
}

float stanley::follow_path(pathing::BasePath& path,
                            controllers::PID* turn,
                            controllers::PID* track,
                            float end_heading, float end_magnitude,
                            int iterations, long long timeout)
{
    bool delete_turn = turn == nullptr;
    bool delete_track = track == nullptr;

    if (delete_turn) {
        turn = new controllers::PID();
        stanley_init_pid(*turn);
    }

    if (delete_track) {
        track = new controllers::PID();
        stanley_init_pid(*track);
    }

    Eigen::Vector2f point;

    auto deriv = [&path, &point](float t) -> float {
        Eigen::Vector2f diff = path.compute(t) - point;
        return diff.dot(path.compute(t, 1)) / diff.norm();
    };

    utils::recompute_path(path, nullptr, nullptr, solvers::Solver::GradientDescent, 1, end_heading, end_magnitude, true);
    std::cout << path.debug_out() << std::endl;

    long long start = pros::millis();
    float t = 0.000;
    while (true) {
        point.noalias() = Eigen::Vector2f(99, 99);
        t = stanley::follow_path_tick(
            path, *turn, *track, deriv, t,
            20
        );

        if (t < 0 || pros::millis() - start > timeout) {
            break;
        }
    }

    printf("Stanley pathing finished at %f\n", t);

    robot::brake();

    if (delete_turn) delete turn;
    if (delete_track) delete track;

    return t;
}

