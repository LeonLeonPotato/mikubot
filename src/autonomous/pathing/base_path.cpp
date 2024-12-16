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

float BasePath::time_parameter(const float s, int b_off, int e_off) const {
    if (s > lengths.back()) return maxt();
    int i = std::lower_bound(lengths.begin() + b_off, lengths.end() - e_off, s) - lengths.begin();
    return (float) i / (lengths.size() - 1) * (points.size() - 1);
}

float BasePath::arc_parameter(const float t) const {
    return lengths[(int) roundf((t / (points.size() - 1) * (lengths.size() - 1)))];
}

void BasePath::solve_lengths(int resolution) {
    Eigen::VectorXf t = Eigen::VectorXf::LinSpaced(resolution+1, 0, points.size() - 1);
    lengths.clear();
    lengths.resize(resolution + 1);
    lengths[0] = 0;
    Eigen::Matrix2Xf res = compute(t);
    for (int i = 1; i < resolution + 1; i++) {
        lengths[i] = lengths[i - 1] + (res.col(i) - res.col(i - 1)).norm();
    }
}

#include "api.h"
void BasePath::profile_path(const ProfileParams& params) {
    Eigen::VectorXf t = Eigen::VectorXf::LinSpaced(params.resolution+1, 0, maxt());
    lengths.clear();
    lengths.resize(params.resolution + 1);
    lengths[0] = 0;
    long long cmtime = pros::micros();
    Eigen::Matrix2Xf res;
    res.resize(2, t.size());
    compute(t, res);
    printf("Computed path in %lld us\n", pros::micros() - cmtime);

    profile.clear();
    profile.emplace_back(0, 0, curvature(0), 0, params.start_v, 0);

    int i = 1, j = 0;
    float s = params.ds;
    while (i < params.resolution) {
        for (; i <= params.resolution; i++) {
            lengths[i] = lengths[i - 1] + (res.col(i) - res.col(i - 1)).norm();
            if (lengths[i] >= s) break;
        }
        if (i > params.resolution) i = params.resolution;
        float ds = lengths[i] - lengths[j];

        float time_param = (float) i / params.resolution * maxt();
        float curve = curvature(time_param);
        float scale = 1 + fabs(curve) * params.track_width / 2.0f;
        float ecv = profile.back().center_v * scale;
        float cv = std::clamp(
            sqrtf(ecv*ecv + 2*params.accel*ds), 
            -params.max_speed, 
            params.max_speed
        ) / scale;

        profile.emplace_back(lengths[i], time_param, curve, 0, cv, 0, 0);
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
        ProfilePoint& lp = profile[i + 1];
        ProfilePoint& p = profile[i];

        float scale = p.curvature * params.track_width / 2.0f;
        float ds = lp.s - p.s;
        float ecv = lp.center_v * (1 + fabs(scale));
        float cv = std::clamp(
            sqrtf(ecv*ecv + 2*params.decel*ds), 
            -params.max_speed, 
            params.max_speed
        ) / (1 + fabs(scale));

        p.center_v = fmin(cv, p.center_v);
        p.left_v = std::clamp(p.center_v * (1 + scale), -params.max_speed, params.max_speed);
        p.right_v = std::clamp(p.center_v * (1 - scale), -params.max_speed, params.max_speed);
        p.angular_v = (p.left_v - p.right_v) / 2.0f;
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
    return (d1.coeffRef(0) * d2.coeffRef(1) - d1.coeffRef(1) * d2.coeffRef(0)) / (d1.dot(d1) * d1.norm() + 1e-6);
}