#include "hardware/odometry/twotracker.h"

using namespace hardware;

void TwoTrackerOdometry::run_task(void) {
    long long iterations = 0;
    long long ltime = pros::millis();
    float ll = rad(side_encoder.get_position() / 100.0f);
    float lh = rad(back_encoder.get_position() / 100.0f);
    float ltheta = rad(imu.get_rotation());

    pros::delay(10);
    while (true) {
        pros::c::mutex_take(tracking_pose_mutex, TIMEOUT_MAX);

        double dt = (pros::millis() - ltime) / 1e3f;
        ltime = pros::millis();

        float ctheta = rad(imu.get_rotation());
        tracking_pose.set_theta(ctheta);
        float dtheta = ctheta - ltheta;
        ltheta = tracking_pose.get_theta();

        float cl = rad(side_encoder.get_position() / 100.0);
        float ch = rad(back_encoder.get_position() / 100.0);
        float travel_lateral = (cl - ll) * tracking_wheel_radius;
        float travel_horizontal = (ch - lh) * tracking_wheel_radius;
        ll = cl;
        lh = ch;

        if (dtheta != 0) {
            float constant = sinc(dtheta / 2);
            travel_lateral = constant * (travel_lateral / dtheta + lateral_tracking_wheel_offset);
            travel_horizontal = constant * (travel_horizontal / dtheta + horizontal_tracking_wheel_offset);
        }

        float av_theta = ctheta - dtheta / 2;

        Eigen::Vector2f travel = {
            travel_lateral * sinf(av_theta) - travel_horizontal * cosf(av_theta),
            travel_lateral * cosf(av_theta) + travel_horizontal * sinf(av_theta)
        };

        tracking_pose += travel;

        iterations++;
        pros::c::mutex_give(tracking_pose_mutex);

        pros::delay(10);
    }
}