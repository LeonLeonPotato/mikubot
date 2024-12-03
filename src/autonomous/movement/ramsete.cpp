#include "autonomous/movement/ramsete.h"
#include "essential.h"

using namespace movement;

TickResult Ramsete::tick(pathing::BasePath& path, const MovementParams& params, PIDGroup pids, 
    const solvers::FunctionGroup& funcs, float t) const
{
    TickResult result;

    int i = path.arc_parameter(t);
    Eigen::Vector2f goal; Eigen::Vector2f deriv;
    while (true) {
        const pathing::ProfilePoint& p = path.get_profile()[i];
        deriv = path(p.t, 1); goal = path(p.t);
        if (deriv.dot(goal - robot::pos) >= 0) {
            break;
        }

        if (i < path.get_profile().size() - 1) {
            i++;
        } else {
            break;
        }
    }

    Eigen::Vector2f crosstrack = goal - robot::pos;
    Eigen::Matrix2f rotator;
    const float rotation_angle = robot::theta + params.reversed * M_PI;
    rotator << cosf(rotation_angle), -sinf(rotation_angle),
        sinf(rotation_angle), cosf(rotation_angle);
    Eigen::Vector2f crosstrack_local = rotator * crosstrack;
    float angle = atan2(deriv.x(), deriv.y());
    float angle_local = robot::angular_diff(angle, params.reversed);
    
    const pathing::ProfilePoint& p = path.get_profile()[i];
    const float zeta = ((const RamseteParams&) params).zeta;
    const float beta = ((const RamseteParams&) params).beta;
    const float angular = p.angular_v * (2 * std::signbit(params.reversed) - 1);
    const float gain = 2 * zeta
        * sqrtf(p.angular_v * p.angular_v
             + beta * p.center_v * p.center_v);

    float v = p.center_v * cosf(angle_local)
         + gain * crosstrack_local.x();

    float w = angular 
        + gain * angle_local
             + beta * p.center_v * sinf(angle_local) * crosstrack_local.y() / (angle_local + 1e-6);

    float motor_v = v / robot::DRIVETRAIN_WHEEL_RADIUS;
    float motor_w = w / robot::DRIVETRAIN_WHEEL_RADIUS;
    if (params.reversed) motor_v *= -1;

    robot::velo(
        motor_v - motor_w,
        motor_v + motor_w
    );

    return {ExitCode::SUCCESS, path.get_profile()[i].t, crosstrack.norm()};
}

MovementResult Ramsete::follow_path_cancellable(volatile bool& cancel_ref, pathing::BasePath& path, 
    const MovementParams& params, PIDGroup pids) const
{
    const solvers::FunctionGroup dummy_funcs;
    const int start_t = pros::millis();

    float t = 0, error = 0;
    while (robot::distance(path.points.back()) > params.goal_threshold) {
        if (cancel_ref) {
            return {
                ExitCode::CANCELLED,
                pros::millis() - start_t,
                0, 0,
                t, 0;
            }
        }

        TickResult result = tick(path, params, pids, funcs, t);
        if (result.exit_code != ExitCode::SUCCESS) {
            return {result.exit_code, result.t};
        }

        t = result.t;
    }

    return {ExitCode::SUCCESS, path.maxt()};
}