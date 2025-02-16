#include "autonomous/pathing/base_path.h"
#include "ansicodes.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>

using namespace pathing;

void BasePath::set_relative(const Eigen::Vector2f& p) {
    for (auto& point : points) {
        point += p;
    }
}

float BasePath::time_parameter(const float s, int b_off, int e_off) const {
    if (s > lengths.back()) return maxt();
    int i = std::lower_bound(lengths.begin() + b_off, lengths.end() - e_off, s) - lengths.begin();
    return (float) i / (lengths.size() - 1) * (points.size() - 1);
}

float BasePath::arc_parameter(const float t) const {
    return lengths[(int) roundf((t / (points.size() - 1) * (lengths.size() - 1)))];
}

void BasePath::solve_lengths(int resolution) {
    lengths.clear();
    lengths.resize(resolution + 1);
    lengths[0] = 0;

    Eigen::MatrixX2f res(2, resolution+1);
    compute(
        Eigen::VectorXf::LinSpaced(resolution+1, 0, points.size() - 1),
        res,
        0
    );

    Eigen::ArrayXf differences = (res.block(0, 1, 2, resolution)
        - res.block(0, 0, 2, resolution))
            .colwise().norm();

    for (int i = 1; i < resolution + 1; i++) {
        lengths[i] = lengths[i - 1] + (res.col(i) - res.col(i - 1)).norm();
    }
}

static float max_speed(const ProfileParams& params, float curve) {
    float max_turn_speed = ((2 * params.max_speed / params.track_width) * params.max_speed) 
        / (fabsf(curve) * params.max_speed + (2 * params.max_speed / params.track_width));
    if (fabsf(curve) < 1e-3) {
        return max_turn_speed;
    }
    float max_slip_speed = sqrtf(params.friction_coeff * (1 / fabsf(curve)) * 981);
    return fminf(max_slip_speed, max_turn_speed);
}

void BasePath::profile_path(const ProfileParams& params) {
    profile.clear();
    profile.reserve(5000);

    Eigen::Vector2f deriv = compute(0, 1);
    float curve = curvature(0);
    profile.push_back(ProfilePoint {compute(0), deriv, atan2f(deriv[0], deriv[1]), 0, 0, 0, curve, params.start_v, 0, params.start_v * curve, 0});

    while (profile.back().path_param <= maxt()) {
        auto& last = profile.back();

        float pathtime = last.path_param + params.ds / last.deriv.norm();
        float curve = curvature(pathtime);
        float angular_vel = curve * last.center_v;
        float angular_accel = (angular_vel - last.angular_v) * (last.center_v / params.ds);

        float max_accel = params.accel - fabsf(angular_accel * params.track_width / 2);
        max_accel = fmaxf(0.01, max_accel);

        float vel = fminf(max_speed(params, curve), sqrtf(last.center_v * last.center_v + 2 * max_accel * params.ds));
        float dist = last.distance + params.ds;

        Eigen::Vector2f d = compute(pathtime, 1);
        profile.push_back(
            {compute(pathtime), d, atan2f(d[0], d[1]), dist, pathtime, 0, curve, vel, 0, angular_vel, 0}
        );
    }

    profile.back().center_v = params.end_v;

    for (int i = profile.size() - 2; i > 0; i--) {
        auto& lst = profile[i + 1];
        auto& cur = profile[i];
        auto& nxt = profile[i-1];

        float angular_accel = (nxt.angular_v - cur.angular_v) * (cur.center_v / params.ds);
        float max_accel = params.decel - fabsf(angular_accel * params.track_width / 2);
        max_accel = fmaxf(0.01, max_accel);
        float vel = fminf(max_speed(params, cur.curvature), sqrtf(lst.center_v * lst.center_v + 2 * max_accel * params.ds));

        cur.center_v = fminf(cur.center_v, vel);
        cur.angular_v = cur.curvature * cur.center_v;
        cur.angular_a = (lst.angular_v - cur.angular_v) * (lst.center_v / params.ds);
        cur.center_a = (lst.center_v - cur.center_v) * (lst.center_v / params.ds);
    }

    for (int i = 0; i < profile.size() - 1; i++) {
        auto& cur = profile[i];
        auto& nxt = profile[i+1];

        float reciprocal_dt = cur.center_v / params.ds;
        if (std::isnan(reciprocal_dt)) {
            reciprocal_dt = 0;
        }
        

        cur.center_a = (nxt.center_v - cur.center_v) * (cur.center_v / dt);
        cur.angular_a = (nxt.angular_v - cur.angular_v) * (cur.center_v / dt);
    }

    profile.front().center_v = params.start_v;
    profile.front().angular_v = profile[1].angular_v;
    profile.front().center_a = (profile[1].center_v - profile.front().center_v) * (profile[1].center_v / params.ds);
    profile.front().angular_a = (profile[1].angular_v - profile.front().angular_v) * (profile[1].center_v / params.ds);
}

ProfilePoint BasePath::profile_point(const float s) const {
    int i = std::lower_bound(profile.begin(), profile.end(), s, [](const ProfilePoint& p, const float s) {
        return p.distance < s;
    }) - profile.begin();
    return profile[i];
}

const std::vector<ProfilePoint>& BasePath::get_profile(void) const {
    return profile;
}

void BasePath::full_sample(int resolution, Eigen::MatrixX2f& res, int deriv) const {
    compute(Eigen::ArrayXf::LinSpaced(resolution, 0, maxt()), res, deriv);
}

void BasePath::compute(const Eigen::ArrayXf& t, Eigen::MatrixX2f& res, int deriv) const {
    Eigen::Vector2f r;
    for (int i = 0; i < t.size(); i++) {
        compute(t(i), r, deriv);
        res.row(i) = r;
    }
}


Eigen::MatrixX2f BasePath::compute(const Eigen::ArrayXf& t, int deriv) const {
    Eigen::MatrixX2f x(t.size(), 2);
    compute(t, x, deriv);
    return x;
}

Eigen::Vector2f BasePath::compute(const float t, int deriv) const {
    Eigen::Vector2f x;
    compute(t, x, deriv);
    return x;
}

Eigen::Vector2f BasePath::normal(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return Eigen::Vector2f(-d(1), d(0));
}

float BasePath::angle(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return atan2f(d(0), d(1));
}

float BasePath::angular_velocity(float t) const {
    Eigen::Vector2f d1 = compute(t, 1);
    Eigen::Vector2f d2 = compute(t, 2);
    return (d1(0) * d2(1) - d1(1) * d2(0)) / (d1.dot(d1) + 1e-6);
}

float BasePath::curvature(float t) const {
    const Eigen::Vector2f d1 = compute(t, 1);
    const Eigen::Vector2f d2 = compute(t, 2);
    return (d1(0) * d2(1) - d1(1) * d2(0)) / ((d1(0) * d1(0) + d1(1) * d1(1)) * 
        sqrtf(d1(0) * d1(0) + d1(1) * d1(1)) + 1e-6);
}