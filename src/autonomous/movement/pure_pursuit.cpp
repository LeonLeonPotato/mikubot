#include "autonomous/movement/pure_pursuit.h"
#include "autonomous/movement/base_movement.h"
#include "essential.h"

using namespace movement;

float recompute_path(pathing::BasePath& path, int goal_i) {
    if (goal_i > 1) {
        for (int i = goal_i; i < path.points.size(); i++) {
            path.points[i - goal_i + 1] = path.points[i];
        }
        for (int i = 0; i < goal_i - 1; i++) {
            path.points.pop_back();
        }
    }

    path.points[0] = Eigen::Vector2f(robot::x, robot::y);
}

// TODO: add curvature based adaptive base speed
float pure_pursuit::follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, float radius,
                        solvers::func_t func, solvers::func_t deriv, int iterations) 
{
    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    auto intersect = solvers::newton(
        func, deriv,
        t, t, path.points.size() - 1, iterations
    );
    t = intersect.first;

    Eigen::Vector2f res = path.compute(t);

    float dtheta = robot::angular_diff(res);
    pid.register_error(fabs(dtheta));

    float dist = robot::distance(res);
    dist = fmin(dist * movement::variables::distance_coeff, radius) / radius * 127;

    float ctrl = pid.get();

    robot::volt(
        (int) (dist + ctrl * dtheta),
        (int) (dist - ctrl * dtheta)
    );
    return t;
}

void follow_path(pathing::BasePath& path, float radius, int iterations) {
    controllers::PID pid; init_pid(pid);

    

}

