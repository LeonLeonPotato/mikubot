#include "autonomous/pathing/base_path.h"
#include "ansicodes.h"
#include "pros/rtos.hpp"
#include <iostream>

using namespace pathing;

void BasePath::set_relative(const Eigen::Vector2f& p) {
    for (auto& point : points) {
        point += p;
    }
}

std::pair<float, float> BaseParams::start_cartesian() const {
    return std::make_pair(
        start_magnitude * sinf(start_heading), 
        start_magnitude * cosf(start_heading)
    );
}

std::pair<float, float> BaseParams::end_cartesian() const {
    return std::make_pair(
        end_magnitude * sinf(end_heading), 
        end_magnitude * cosf(end_heading)
    );
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

void BasePath::profile_path(const ProfileParams& params) {
    lengths.clear();
    lengths.resize(params.resolution + 1);
    lengths[0] = 0;

    // auto start_t = pros::micros();
    // Eigen::Matrix2Xf res(2, params.resolution+1);
    // compute(Eigen::VectorXf::LinSpaced(params.resolution+1, 0, maxt()), res);
    // printf("[2D MP] Computed path in %lld us\n", pros::micros() - start_t);

    int start_t = pros::millis();
    Eigen::MatrixX2f res(params.resolution+1, 2);
    full_sample(
        params.resolution+1, 
        res, 
    0);
    int diff = pros::millis() - start_t;

    std::cout << PREFIX << "Computed path in " << diff << " ms\n";

    start_t = pros::millis();
    Eigen::ArrayXf differences = 
        (res.block(1, 0, params.resolution, 2)
            - res.block(0, 0, params.resolution, 2))
                .rowwise().norm();
    std::cout << PREFIX << "Computed differences in " << pros::millis() - start_t << " ms\n";

    profile.clear();
    profile.reserve(2500);
    profile.emplace_back(compute(0), angle(0), 0, 0, curvature(0), 0, params.start_v, 0);

    start_t = pros::millis();

    int i = 1, j = 0;
    float s = params.ds;
    while (i < params.resolution) {
        for (; i <= params.resolution; i++) {
            lengths[i] = lengths[i - 1] + differences[i - 1];
            if (lengths[i] >= s) break;
        }
        if (i > params.resolution) i = params.resolution;
        const float ds = lengths[i] - lengths[j];

        const float time_param = (float) i / params.resolution * maxt();

        const Eigen::Vector2f d0 = compute(time_param, 0);
        const Eigen::Vector2f d1 = compute(time_param, 1);
        const Eigen::Vector2f d2 = compute(time_param, 2);

        const float curve = (d1(0) * d2(1) - d1(1) * d2(0)) / ((d1(0) * d1(0) + d1(1) * d1(1)) * 
            sqrtf(d1(0) * d1(0) + d1(1) * d1(1)) + 1e-6);
        const float scale = 1 + fabsf(curve) * params.track_width / 2.0f;
        const float ecv = profile.back().center_v * scale;
        const float cv = std::clamp(
            sqrtf(ecv*ecv + 2*params.accel*ds), 
            -params.max_speed, 
            params.max_speed
        ) / scale;

        profile.emplace_back(d0, atan2f(d1(0), d1(1)), lengths[i], time_param, curve, 0, cv, 0, 0);
        s += params.ds;
        j = i;
    }

    // float __scale = profile.back().curvature * params.track_width / 2.0f;
    // profile.back().center_v = params.end_v;
    // profile.back().left_v = std::clamp(params.end_v * (1 - __scale), -params.max_speed, params.max_speed);
    // profile.back().right_v = std::clamp(params.end_v * (1 + __scale), -params.max_speed, params.max_speed);
    // profile.back().angular_v = (profile.back().left_v - profile.back().right_v) / 2.0f;
    profile.back().center_v = params.end_v;
    profile.back().left_v = params.end_v;
    profile.back().right_v = params.end_v;
    profile.back().angular_v = 0;

    for (int i = profile.size() - 2; i > 0; i--) {
        const ProfilePoint& lp = profile[i + 1];
        ProfilePoint& p = profile[i];

        const float scale = p.curvature * params.track_width / 2.0f;
        const float ds = lp.s - p.s;
        const float ecv = lp.center_v * (1 + fabsf(scale));
        const float cv = std::clamp(
            sqrtf(ecv*ecv + 2*params.decel*ds), 
            -params.max_speed, 
            params.max_speed
        ) / (1 + fabsf(scale));

        p.center_v = fminf(cv, p.center_v);
        p.left_v = std::clamp(p.center_v * (1 + scale), -params.max_speed, params.max_speed);
        p.right_v = std::clamp(p.center_v * (1 - scale), -params.max_speed, params.max_speed);
        p.angular_v = (p.left_v - p.right_v) / 2.0f;
    }

    diff = pros::millis() - start_t;
    std::cout << PREFIX << "Computed profile in " << diff << " ms\n";
}

ProfilePoint BasePath::profile_point(const float s) const {
    int i = std::lower_bound(profile.begin(), profile.end(), s, [](const ProfilePoint& p, const float s) {
        return p.s < s;
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