#include "autonomous/movement.h"
#include "essential.h"

#include "api.h"

namespace movement {
namespace variables {
    float distance_coeff = 5.0;

    float turning_kP = 300;
    float turning_kI = 1;
    float turning_kD = 0;

    float turning_I_disable_min = -1;
    float turning_I_disable_max = 1;
    float turning_I_min = -50;
    float turning_I_max = 50;
};

void init_pid(controllers::PID& pid) {
    pid = controllers::PID(
        variables::turning_kP, 
        variables::turning_kI, 
        variables::turning_kD, 
        variables::turning_I_min, 
        variables::turning_I_max, 
        variables::turning_I_disable_min, 
        variables::turning_I_disable_max
    );
}

void goto_pos_tick(const Eigen::Vector2f& point, controllers::PID& pid) {
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

void turn_towards_tick(float angle, controllers::PID& pid) {
    float theta_diff = robot::angular_diff(angle);
    pid.register_error(fabs(theta_diff));
    float turn_vel = pid.get();
    robot::volt(
        theta_diff * turn_vel, 
        -theta_diff * turn_vel
    );
}

void goto_pos(const Eigen::Vector2f& point, float threshold) {
    controllers::PID pid;
    init_pid(pid);
    
    while (robot::distance(point) > threshold) {
        goto_pos_tick(point, pid);
        pros::delay(20);
    }
}

void turn_towards(float angle, float threshold) {
    controllers::PID pid;
    init_pid(pid);

    while (fabs(robot::angular_diff(angle)) > threshold) {
        turn_towards_tick(angle, pid);
        pros::delay(20);
    }
}

void goto_pose(const Eigen::Vector2f& point, float angle, float threshold, float theta_threshold) {
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
} // namespace pathing