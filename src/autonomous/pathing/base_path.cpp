#include "autonomous/pathing/base_path.h"

using namespace pathing;

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

void BasePath::solve_lengths(int resolution) {
    Eigen::VectorXf t = Eigen::VectorXf::LinSpaced(resolution, 0, points.size() - 1);
    lengths.resize(resolution + 1);
    lengths[0] = 0;
    Eigen::Matrix2Xf res; compute(t, res);
    for (int i = 1; i < resolution + 1; i++) {
        lengths[i] = lengths[i - 1] + (res.col(i) - res.col(i - 1)).norm();
    }
}

float BasePath::time_parameter(const float s) const {
    int i = std::lower_bound(lengths.begin(), lengths.end(), s) - lengths.begin();
    return (float) i / lengths.size() * (points.size() - 1);
}

float BasePath::arc_parameter(const float t) const {
    return lengths[(int) (t / (points.size() - 1) * lengths.size())];
}