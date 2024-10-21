#include "autonomous/movement/pure_pursuit.h"
#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/numerical_solvers.h"
#include "essential.h"

using namespace movement;

float func(pathing::BasePath& path, const Eigen::Vector2f& pos, float t) {
    Eigen::Vector2f res = path.compute(t);
    return (res - pos).squaredNorm();
}

// float pure_pursuit::follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, int iterations) {
//     Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
//     auto intersect = solvers::newton(
//         path, point, 
//         radius, t, 0, path.points.size() - 1, iterations);
//     t = intersect.first;

//     Eigen::Vector2f res = path.compute(t);

//     float dtheta = robot::angular_diff(res);
//     pid.register_error(fabs(dtheta));

//     float dist = robot::distance(res);
//     dist = fmin(dist * movement::variables::distance_coeff, radius) / radius * 127;

//     float ctrl = pid.get();
//     int left = (int) (dist + ctrl * dtheta);
//     int right = (int) (dist - ctrl * dtheta);

//     robot::volt(left, right);
//     return t;
// }

