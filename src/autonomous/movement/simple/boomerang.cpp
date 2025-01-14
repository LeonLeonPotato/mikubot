#include "autonomous/movement/simple/boomerang.h"
#include "ansicodes.h"
#include "essential.h"
#include "gui/debugscreen.h"
#include "pros/rtos.h"

static inline float safe_sinc(float x) {
    if (fabsf(x) < 1e-3) return 1 - (x*x/6.0f) + (x*x*x*x/120.0f);
    return sinf(x) / x;
}

using namespace movement;

DEFINE_TICK(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
{
    const float true_target_dist = robot::distance(point);
    const Eigen::Vector2f carrot = point
         - lead * true_target_dist * Eigen::Vector2f(sinf(angle), cosf(angle));
    float theta_error = robot::angular_diff(carrot, params.reversed);

    Eigen::Rotation2D<float> rot(robot::theta);
    Eigen::Vector2f error = rot * (carrot - robot::pos);

    if (true_target_dist < 10) {
        float scale = true_target_dist / 10;
        theta_error = scale * theta_error + (1 - scale) * robot::angular_diff(angle, params.reversed);
    }

    debugscreen::debug_message = "Carrot: [" + std::to_string(carrot.x()) + ", " + std::to_string(carrot.y()) + "]\n";
    debugscreen::debug_message += "Angle diff: " + std::to_string(theta_error) + "\n";

    printf("Carrot: %f, %f Angular diff: %f | true diff: %f\n", 
        carrot.x(), carrot.y(), theta_error, 
        robot::angular_diff(angle, params.reversed));

    // printf("%sCarrot: [%f, %f]\n", PREFIX.c_str(), carrot.x(), carrot.y());

    // float speed = pids.linear.get(robot::distance(carrot));
    // float turn = pids.angular.get(theta_error);
    // if (params.reversed) speed *= -1;
    // if (params.use_cosine_scaling) speed *= cosf(theta_error);
    // speed = std::clamp(speed, -params.max_linear_speed, params.max_linear_speed);

    // // printf("%s%f, %f\n", PREFIX.c_str(), speed, turn);
    // robot::velo(speed + turn, speed - turn);
    float klat = 2.0f;

    float vd = pids.linear.get(error.norm() * (cosf(theta_error) > 0 ? 1 : -1));
    float omega = pids.angular.get(theta_error) + klat * vd * error.y() * safe_sinc(theta_error);
    float v = vd * fabsf(cosf(theta_error));

    if (params.reversed) v = -v;

    robot::velo(v + omega, v - omega);

    return { ExitCode::SUCCESS, true_target_dist, 0 };
}

DEFINE_CANCELLABLE(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
{
    const int start = pros::millis();
    SimpleResult last_tick;
    while (robot::distance(point) > params.linear_exit_threshold && 
           fabs(robot::angular_diff(angle, params.reversed)) > params.angular_exit_threshold) 
    {
        if (cancel_ref) {
            return { ExitCode::CANCELLED, last_tick.error, __timediff(start) };
        }

        if (pros::millis() - start > params.timeout) {
            return { ExitCode::TIMEOUT, last_tick.error, __timediff(start) };
        }

        last_tick = boomerang_tick(point, angle, lead, params, pids);
        if (last_tick.code != ExitCode::SUCCESS) {
            return { last_tick.code, last_tick.error, __timediff(start) };
        }

        pros::c::task_delay(20);
    }

    return { ExitCode::SUCCESS, last_tick.error, __timediff(start) };
}

DEFINE_STANDARD(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
{
    bool cancel = false;
    return boomerang_cancellable(point, angle, lead, params, pids, cancel);
}

DEFINE_ASYNC(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)
{
    Future<SimpleResult> future;
    pros::Task task([&point, angle, lead, params, pids, &future] () {
        future.set_value(boomerang_cancellable(
            point, angle, lead, params, pids, 
            future.get_state()->cancelled
        ));
    });
    return future;
}

