#include "autonomous/movement/ramsete.h"
#include "essential.h"

using namespace movement::ramsete;
using namespace movement;

static float safe_sinc(float x) {
    if (fabsf(x) < 1e-3) return 1 - (x*x/6.0f) + (x*x*x*x/120.0f);
    return sinf(x) / x;
}

static RamseteResult tick(
    pathing::BasePath& path, 
    const RamseteParams& params, 
    int i)
{
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
    const float angular = p.angular_v * (2 * std::signbit(params.reversed) - 1);
    const float gain = 2 * params.zeta
        * sqrtf(p.angular_v * p.angular_v
             + params.beta * p.center_v * p.center_v);

    float v = p.center_v * cosf(angle_local)
         + gain * crosstrack_local.x();

    float w = angular 
        + gain * angle_local
             + params.beta * p.center_v * safe_sinc(angle_local) * crosstrack_local.y();

    float motor_v = v / robot::DRIVETRAIN_WHEEL_RADIUS;
    float motor_w = w / robot::DRIVETRAIN_WHEEL_RADIUS;
    if (params.reversed) motor_v *= -1;
    motor_v = std::clamp(motor_v, -params.max_linear_speed, params.max_linear_speed);

    robot::velo(
        motor_v + motor_w,
        motor_v - motor_w
    );

    return {ExitCode::SUCCESS, crosstrack.norm(), 0, i};
}

RamseteResult ramsete::follow_path_cancellable(    
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const RamseteParams& params)
{
    const int start_t = pros::millis();

    RamseteResult last_tick;
    while (robot::distance(path.points.back()) > params.exit_threshold) {
        if (cancel_ref) {
            return {ExitCode::CANCELLED, last_tick.error, (int) (pros::millis() - start_t), last_tick.i};
        }

        if (pros::millis() - start_t > params.timeout) {
            return {ExitCode::TIMEOUT, last_tick.error, (int) (pros::millis() - start_t), last_tick.i};
        }

        last_tick = tick(path, params, last_tick.i);

        if (last_tick.code != ExitCode::SUCCESS) {
            last_tick.time_taken_ms = pros::millis() - start_t;
            return last_tick;
        }
    }

    return {ExitCode::SUCCESS, last_tick.error, (int) (pros::millis() - start_t), last_tick.i};
}

RamseteResult ramsete::follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const float beta, const float zeta,
    const SimpleMovementParams& params) 
{
    RamseteParams new_params;
    new_params.copy_from(params);
    new_params.beta = beta;
    new_params.zeta = zeta;
    return follow_path_cancellable(cancel_ref, path, new_params);
}