#include "autonomous/movement/simple/boomerang.h"
#include "ansicodes.h"
#include "essential.h"
#include "gui/debugscreen.h"
#include "pros/rtos.h"

using namespace movement;

DEFINE_TICK(boomerang, PIDGroup,
    const Pose& pose,
    const float lead)
{
    const float true_target_dist = robot::distance(pose);
    const Eigen::Vector2f carrot = pose.pos()
         - lead * true_target_dist * Eigen::Vector2f(sinf(pose.theta()), cosf(pose.theta()));
    float theta_error = robot::angular_diff(carrot, params.reversed);

    Eigen::Rotation2D<float> rot(robot::theta());
    Eigen::Vector2f error = rot * (carrot - robot::pos()) / 100.0f;

    if (true_target_dist < 7.5) {
        float scale = true_target_dist / 7.5;
        theta_error = robot::angular_diff(pose.theta(), params.reversed);
        error = rot * (pose.pos() - robot::pos()) / 100.0f;
    }

    debugscreen::debug_message = "Carrot: [" + std::to_string(carrot.x()) + ", " + std::to_string(carrot.y()) + "]\n";
    debugscreen::debug_message += "Angle diff: " + std::to_string(theta_error) + "\n";

    // printf("%sCarrot: [%f, %f]\n", PREFIX.c_str(), carrot.x(), carrot.y());

    // float speed = pids.linear.get(robot::distance(carrot));
    // float turn = pids.angular.get(theta_error);
    // if (params.reversed) speed *= -1;
    // if (params.use_cosine_scaling) speed *= cosf(theta_error);
    // speed = std::clamp(speed, -params.max_linear_speed, params.max_linear_speed);

    // // printf("%s%f, %f\n", PREFIX.c_str(), speed, turn);
    // robot::velo(speed + turn, speed - turn);
    float klat = 0.0f;

    float vd = pids.linear.get(error.norm() * (cosf(theta_error) > 0 ? 1 : -1) * 100.0f);
    float omega = pids.angular.get(theta_error) + klat * vd * error.y() * sinc(theta_error);
    float v = vd * fabsf(cosf(theta_error));
    v = std::clamp(v, -params.max_linear_speed, params.max_linear_speed);

    if (params.reversed) v = -v;

    debugscreen::debug_message += "VD: " + std::to_string(vd) + "\n";
    debugscreen::debug_message += "Omega: " + std::to_string(omega) + "\n";
    debugscreen::debug_message += "V: " + std::to_string(v) + "\n";
    debugscreen::debug_message += "Error: " + std::to_string(error.x()) + ", " + std::to_string(error.y()) + "\n";

    float left = v + omega;
    float right = v - omega;
    // auto max_abs_val = std::max(std::abs(left), std::abs(right));
    // if (max_abs_val > 1) {
    //     left /= max_abs_val;
    //     right /= max_abs_val;
    // }
    robot::velo(left, right);

    return { ExitCode::SUCCESS, true_target_dist, theta_error, 0 };
}

DEFINE_CANCELLABLE(boomerang, PIDGroup,
    const Pose& pose,
    const float lead)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (fabsf(last_tick.angular_error) > params.angular_exit_threshold || last_tick.linear_error > params.linear_exit_threshold) {
        if (cancel_ref) {
            last_tick.code = ExitCode::CANCELLED;
            break;
        }

        if (__timediff(start) >= params.timeout) {
            last_tick.code = ExitCode::TIMEOUT;
            break;
        }

        last_tick = boomerang_tick(pose, lead, params, pids);

        if (last_tick.code != ExitCode::SUCCESS) {
            break;
        }

        pros::delay(params.delay);
    }

    last_tick.time_taken_ms = __timediff(start);
    pids.reset();
    return last_tick;
}

DEFINE_STANDARD(boomerang, PIDGroup,
    const Pose& pose,
    const float lead)
{
    bool cancel = false;
    return boomerang_cancellable(pose, lead, params, pids, cancel);
}

DEFINE_ASYNC(boomerang, PIDGroup,
    const Pose& pose,
    const float lead)
{
    Future<SimpleResult> future;
    pros::Task task([future, &params, pids, &pose, lead] () mutable {
        robot::chassis.take_drive_mutex();
        future.set_value(boomerang_cancellable(
            pose, lead, params, pids, 
            future.get_state()->cancelled
        ));
        robot::chassis.give_drive_mutex();
    });
    return future;
}

