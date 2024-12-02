#include "autonomous/movement/ramsete.h"
#include "essential.h"

using namespace movement;

TickResult Ramsete::tick(pathing::BasePath& path, const MovementParams& params, PIDGroup pids, 
    const solvers::FunctionGroup& funcs, float t) const
{
    TickResult result;

    int i = path.arc_parameter(t);
    Eigen::Vector2f goal;
    while (true) {
        const pathing::ProfilePoint& p = path.get_profile()[i];
        goal = path(p.t);
        if (path(p.t, 1).dot(robot::pos - goal) >= 0) {
            break;
        }
        i++;
    }

    Eigen::Vector2f crosstrack = goal - robot::pos;
    Eigen::Matrix2f rotator;
    rotator << cosf(robot::theta + params.reversed * M_PI), -sinf(robot::theta + params.reversed * M_PI),
        sinf(robot::theta + params.reversed * M_PI), cosf(robot::theta + params.reversed * M_PI);
    Eigen::Vector2f crosstrack_local = rotator * crosstrack;
    float angular_diff = robot::angular_diff(goal, params.reversed);
}