#include "autonomous/movement/ramsete.h"
#include "Eigen/src/Core/Matrix.h"
#include "autonomous/pathing/base_path.h"
#include "essential.h"
#include "mathtils.h"
#include "pose.h"
#include <cstdio>
#include <math.h>
#include <string>
#include <vector>

using namespace movement::ramsete;
using namespace movement;

static RamseteResult tick(
    pathing::BasePath& path, 
    const RamseteParams& params, 
    int i)
{
    Eigen::Vector2f goal; Eigen::Vector2f deriv;
    while (true) {
        const pathing::ProfilePoint& p = path.get_profile()[i];
        deriv = path(p.t, 1);

        if (deriv.dot(p.pos - robot::pos()) > 0) {
            break;
        }

        if (i < path.get_profile().size() - 1) {
            i++;
        } else {
            i = path.get_profile().size() - 1;
            break;
        }
    }

    const pathing::ProfilePoint& p = path.get_profile()[i];

    const float rotation_angle = robot::theta() + params.reversed * M_PI;
    Eigen::Vector2f crosstrack = (goal - robot::pos()) / 100.0f;
    Eigen::Vector2f crosstrack_local = Eigen::Rotation2Df(rotation_angle) * crosstrack;
    float angle_local = robot::angular_diff(p.heading, params.reversed);
    // printf("Deriv Theta: %f | Robot theta: %f | Angle local: %f\n", angle, robot::theta, angle_local);
    
    // Might not be the case, reversed ramsete has not been tested
    const float angular = p.angular_v * (2*params.reversed - 1);

    const float gain = 2 * params.zeta
        * sqrtf(p.angular_v * p.angular_v
             + params.beta * p.center_v * p.center_v);

    float v = p.center_v * cosf(angle_local) + gain * crosstrack_local.y();

    float w = angular
        + gain * angle_local
            + params.beta * p.center_v * sinc(angle_local) * crosstrack_local.x();

    // printf("Terms: %f, %f, %f, %f\n", angular, gain*angle_local, params.beta * p.center_v * sinf(angle_local), params.beta * crosstrack_local.x());

    const float max_rads_per_second = robot::max_speed() * M_TWOPI / 60.0f;
    float motor_v = (v / robot::DRIVETRAIN_LINEAR_MULT) / max_rads_per_second;
    float motor_w = (w / robot::DRIVETRAIN_LINEAR_MULT) / max_rads_per_second;
    if (params.reversed) motor_v *= -1;
    motor_v = std::clamp(motor_v, -params.max_linear_speed, params.max_linear_speed);

    robot::velo(
        motor_v + motor_w,
        motor_v - motor_w
    );

    return {ExitCode::SUCCESS, crosstrack.norm(), 0, i};
}

static RamseteResult tick_poses_recording(
    std::vector<pathing::ProfilePoint>& profiled_points, 
    const RamseteParams& params, 
    int i)
{
    Eigen::Vector2f goal; Eigen::Vector2f deriv;
    while (true) {
        const pathing::ProfilePoint& p = profiled_points[i];
        deriv = Eigen::Vector2f(sinf(p.heading), cosf(p.heading));

        if (deriv.dot(p.pos - robot::pos()) > 0) {
            break;
        }

        if (i < profiled_points.size() - 1) {
            i++;
        } else {
            i = profiled_points.size() - 1;
            break;
        }
    }

    const pathing::ProfilePoint& p = profiled_points[i];

    const float rotation_angle = robot::theta() + params.reversed * M_PI;
    Eigen::Vector2f crosstrack = (goal - robot::pos()) / 100.0f;
    Eigen::Vector2f crosstrack_local = Eigen::Rotation2Df(rotation_angle) * crosstrack;
    float angle_local = robot::angular_diff(p.heading, params.reversed);
    // printf("Deriv Theta: %f | Robot theta: %f | Angle local: %f\n", angle, robot::theta, angle_local);
    
    // Might not be the case, reversed ramsete has not been tested
    const float angular = p.angular_v * (2*params.reversed - 1);

    const float gain = 2 * params.zeta
        * sqrtf(p.angular_v * p.angular_v
             + params.beta * p.center_v * p.center_v);

    float v = p.center_v * cosf(angle_local) + gain * crosstrack_local.y();

    float w = angular
        + gain * angle_local
            + params.beta * p.center_v * sinc(angle_local) * crosstrack_local.x();

    const float max_rads_per_second = robot::max_speed() * M_TWOPI / 60.0f;
    float motor_v = (v / robot::DRIVETRAIN_LINEAR_MULT) / max_rads_per_second;
    float motor_w = (w / robot::DRIVETRAIN_LINEAR_MULT) / max_rads_per_second;
    if (params.reversed) motor_v *= -1;
    motor_v = std::clamp(motor_v, -params.max_linear_speed, params.max_linear_speed);

    robot::velo(
        motor_v + motor_w,
        motor_v - motor_w
    );

    return {ExitCode::SUCCESS, crosstrack.norm(), 0, i};
}

static std::vector<std::string> split_string(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

static std::vector<pathing::ProfilePoint> create_from_poses(
    const std::string& filename
) {
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) return {};
    std::vector<pathing::ProfilePoint> profile;

    std::string header(512, '\0');
    fgets(header.data(), header.size(), file);

    char buffer[512];
    int i = 0;
    while (fgets(buffer, sizeof(buffer), file)) {
        if (buffer[0] == '\n') continue;
        pathing::ProfilePoint point;
        std::vector<std::string> tokens = split_string(buffer, ',');
        point.t = std::stoll(tokens[0]) / 1e6f;
        point.pos = Eigen::Vector2f(
            std::stof(tokens[1]), 
            std::stof(tokens[2])
        );
        point.heading = std::stof(tokens[3]);
        point.left_v = std::stof(tokens[4]);
        point.right_v = std::stof(tokens[5]);
        
        point.center_v = (point.left_v + point.right_v) / 2.0f;
        point.angular_v = (point.right_v - point.left_v) / robot::DRIVETRAIN_WIDTH;

        if (i != 0) {
            pathing::ProfilePoint& lp = profile[i-1];
            float dt = point.t - profile[i-1].t;
            lp.left_a = (point.left_v - profile[i-1].left_v) / dt;
            lp.right_a = (point.right_v - profile[i-1].right_v) / dt;
            lp.angular_a = (point.angular_v - profile[i-1].angular_v) / dt;
            lp.center_a = (point.center_v - profile[i-1].center_v) / dt;
        }
        point.left_a = 0;
        point.right_a = 0;
        point.angular_a = 0;
        point.center_a = 0;

        profile.push_back(point);
        i++;
    }

    return profile;
}

RamseteResult ramsete::follow_path_cancellable(    
    volatile bool& cancel_ref, 
    pathing::BasePath& path, 
    const RamseteParams& params)
{
    const int start = pros::millis();

    RamseteResult last_tick {.i = 1};
    while (robot::distance(path.points.back()) > params.linear_exit_threshold || last_tick.i != path.get_profile().size()-1) {
        if (cancel_ref) {
            last_tick.code = ExitCode::CANCELLED;
            break;
        }

        if (__timediff(start) >= params.timeout) {
            last_tick.code = ExitCode::TIMEOUT;
            break;
        }

        last_tick = tick(path, params, last_tick.i);

        if (last_tick.code != ExitCode::SUCCESS) {
            break;
        }

        pros::delay(params.delay);
    }

    last_tick.time_taken_ms = __timediff(start);
    return last_tick;
}

RamseteResult ramsete::follow_poses_recording_cancellable(    
    volatile bool& cancel_ref, 
    const std::string& filename, 
    const RamseteParams& params)
{
    const int start = pros::millis();
    std::vector<pathing::ProfilePoint> profile = create_from_poses(filename);

    RamseteResult last_tick {.i = 1};
    while ((profile.back().pos - robot::pos()).norm() > params.linear_exit_threshold || last_tick.i != profile.size()-1) {
        if (cancel_ref) {
            last_tick.code = ExitCode::CANCELLED;
            break;
        }

        if (__timediff(start) >= params.timeout) {
            last_tick.code = ExitCode::TIMEOUT;
            break;
        }

        last_tick = tick_poses_recording(profile, params, last_tick.i);

        if (last_tick.code != ExitCode::SUCCESS) {
            break;
        }

        pros::delay(params.delay);
    }

    last_tick.time_taken_ms = __timediff(start);
    return last_tick;
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