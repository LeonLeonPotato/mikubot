#include "hardware/odometry/twotracker.h"

using namespace hardware::odometry;

static inline bool is_fucked(float x) {
    return isnan(x) || isinf(x);
}

void TwoTrackerOdometry::run_task(void) {
    long long iterations = 0;
    long long ltime = pros::millis();
    float ll = rad(side_encoder.get_position() / 100.0f);
    float lh = rad(back_encoder.get_position() / 100.0f);
    auto lthetas = imu.get_rotations_eigen();

    float l_left = left_motors.get_position_average() * linear_mult;
    float l_right = right_motors.get_position_average() * linear_mult;

    pros::delay(10);
    while (true) {
        pros::c::mutex_take(tracking_pose_mutex, TIMEOUT_MAX);

        double dt = (pros::millis() - ltime) / 1e3f;
        ltime = pros::millis();

        auto cthetas = imu.get_rotations_eigen();
        auto dthetas = cthetas - lthetas;
        lthetas = cthetas;

        auto c_left = left_motors.get_position_average() * linear_mult;
        auto c_right = right_motors.get_position_average() * linear_mult;
        float d_left = c_left - l_left;
        float d_right = c_right - l_right;
        l_left = c_left;
        l_right = c_right;

        float dtheta = rad(dthetas.sum() / dthetas.size());
        if (isnan(dtheta) || isinf(dtheta) || (fabs(dtheta) > 999999)) {
            dtheta = (d_left - d_right) / track_width;
        }

        float ctheta = tracking_pose.get_theta() + dtheta;
        tracking_pose.set_theta(ctheta);
        lthetas = cthetas;

        float cl = rad(side_encoder.get_position() / 100.0) * tracking_wheel_radius;
        float ch = rad(back_encoder.get_position() / 100.0) * tracking_wheel_radius;
        float travel_forwards = (cl - ll);
        float travel_horizontal = (ch - lh);
        ll = cl;
        lh = ch;

        if (isnan(travel_forwards) || isinf(travel_forwards) || (fabs(travel_forwards) > 999999)) {
            travel_forwards = (d_left + d_right) / 2;
            if (dtheta != 0) {
                float constant = 2 * sin(dtheta / 2);
                travel_forwards = constant * travel_forwards / dtheta;
                travel_horizontal = constant * (travel_horizontal / dtheta + horizontal_tracking_wheel_offset);
            }
        } else {
            if (dtheta != 0) {
                float constant = 2 * sin(dtheta / 2);
                travel_forwards = constant * (travel_forwards / dtheta + forwards_tracking_wheel_offset);
                travel_horizontal = constant * (travel_horizontal / dtheta + horizontal_tracking_wheel_offset);
            }
        }

        float av_theta = ctheta - dtheta / 2;

        Eigen::Vector2f travel = {
            travel_forwards * sinf(av_theta) - travel_horizontal * cosf(av_theta),
            travel_forwards * cosf(av_theta) + travel_horizontal * sinf(av_theta)
        };
        if (isnan(travel(0)) || isnan(travel(1)) || isinf(travel(0)) 
            || isinf(travel(1)) || (fabs(travel(0)) > 999999) || (fabs(travel(1)) > 999999)) 
        {
            travel = {0, 0};
        }

        tracking_pose += travel;

        iterations++;
        pros::c::mutex_give(tracking_pose_mutex);

        pros::delay(10);
    }
}