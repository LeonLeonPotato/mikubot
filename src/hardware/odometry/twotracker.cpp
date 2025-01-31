#include "hardware/odometry/twotracker.h"

using namespace hardware;

void TwoTrackerOdometry::run_task(void) {
    long long iterations = 0;
    long long ltime = pros::millis();
    float ll = rad(side_encoder.get_position() / 100.0f);
    float lh = rad(back_encoder.get_position() / 100.0f);
    std::vector<float> lthetas = imu.get_rotations();

    pros::delay(10);
    while (true) {
        pros::c::mutex_take(tracking_pose_mutex, TIMEOUT_MAX);

        double dt = (pros::millis() - ltime) / 1e3f;
        ltime = pros::millis();

        std::vector<float> cthetas = imu.get_rotations();
        std::vector<float> dthetas = cthetas;
        for (int i = 0; i < cthetas.size(); i++) dthetas[i] -= lthetas[i];
        float dtheta = rad(average(dthetas));
        tracking_pose.set_theta(tracking_pose.get_theta() + dtheta);
        float ctheta = tracking_pose.get_theta();
        lthetas = cthetas;

        float cl = rad(side_encoder.get_position() / 100.0);
        float ch = rad(back_encoder.get_position() / 100.0);
        float travel_lateral = (cl - ll) * tracking_wheel_radius;
        float travel_horizontal = (ch - lh) * tracking_wheel_radius;
        ll = cl;
        lh = ch;

        if (dtheta != 0) {
            float constant = 2 * sin(dtheta / 2);
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