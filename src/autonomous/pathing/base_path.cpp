#include "autonomous/pathing/base_path.h"

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

float BasePath::time_parameter(const float s) const {
    int i = std::lower_bound(lengths.begin(), lengths.end(), s) - lengths.begin();
    return (float) i / (lengths.size() - 1) * (points.size() - 1);
}

float BasePath::arc_parameter(const float t) const {
    return lengths[(int) round((t / (points.size() - 1) * (lengths.size() - 1)))];
}

void BasePath::solve_lengths(int resolution) {
    Eigen::VectorXf t = Eigen::VectorXf::LinSpaced(resolution+1, 0, points.size() - 1);
    lengths.resize(resolution + 1);
    lengths[0] = 0;
    Eigen::Matrix2Xf res = compute(t);
    for (int i = 1; i < resolution + 1; i++) {
        lengths[i] = lengths[i - 1] + (res.col(i) - res.col(i - 1)).norm();
    }
}

void BasePath::profile_path(const ProfileParams& params) {
    solve_lengths(params.resolution);

    profile.reserve((int) ceil(lengths.back() / params.ds) + 1); // safe +1 bc idk how to math it out (thug it out rahhh)

    float center_v = params.start_v;
    for (float s = 0; s <= lengths.back(); s += params.ds) {
        float time_param = time_parameter(s);
        float curve = curvature(time_param);
        float scale = fabs(curve) * params.track_width / 2.0f;
        profile.emplace_back(s, time_param, curve, 0, center_v, 0);
        center_v = std::clamp(sqrtf(center_v*center_v + 2*params.accel*params.ds), 0.0f, params.max_speed / (1 + scale));
        // printf("s: %f, t: %f, curve: %f, center_v: %f d1: (%f, %f)\n", s, time_param, curve, center_v, compute(time_param, 2)(0), compute(time_param, 2)(1));
    }

    center_v = params.end_v;
    for (int i = profile.size() - 1; i > 0; i--) {
        ProfilePoint& p = profile[i];

        float scale = fabs(p.curvature) * params.track_width / 2.0f;
        p.center_v = fmin(p.center_v, center_v);
        p.left_v = std::clamp(p.center_v * (1 + scale), -params.max_speed, params.max_speed);
        p.right_v = std::clamp(p.center_v * (1 - scale), -params.max_speed, params.max_speed);
        center_v = std::clamp(sqrtf(center_v*center_v + 2*params.decel*params.ds), 0.0f, params.max_speed / (1 + scale));
    }
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

Eigen::Vector2f BasePath::normal(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return Eigen::Vector2f(-d(1), d(0));
}

float BasePath::angle(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return atan2(d(0), d(1));
}

float BasePath::angular_velocity(float t) const {
    Eigen::Vector2f d1 = compute(t, 1);
    Eigen::Vector2f d2 = compute(t, 2);
    return (d1(0) * d2(1) - d1(1) * d2(0)) / d1.dot(d1);
}

float BasePath::curvature(float t) const {
    const Eigen::Vector2f d1 = compute(t, 1);
    const Eigen::Vector2f d2 = compute(t, 2);
    return (d1(0) * d2(1) - d1(1) * d2(0)) / (d1.dot(d1) * d1.norm() + 1e-6);
}