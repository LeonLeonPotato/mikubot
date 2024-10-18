#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

namespace pathing {
class PolygonPath : public BasePath {
    public:
        std::vector<Eigen::Vector2f> vertices;

        PolygonPath(void) {}
        PolygonPath(const std::vector<Eigen::Vector2f>& vertices) : vertices(vertices) {}

        void solve_lengths(int resolution = 150) override;
        inline float time_parameter(float s) const override;
        inline float arc_parameter(float t) const override;

        inline void compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv = 0) const override;
        inline Eigen::Matrix<float, 2, -1> compute(const Eigen::VectorXf& t, int deriv = 0) const override;
        inline void compute(float t, Eigen::Vector2f& res, int deriv = 0) const override;
        inline Eigen::Vector2f compute(float t, int deriv = 0) const override;

        inline Eigen::Vector2f normal(float t) const override;
        inline float angle(float t) const override;
};
}

float pathing::PolygonPath::time_parameter(float s) const {
    int i = std::lower_bound(lengths.begin(), lengths.end(), s) - lengths.begin();
    return (float) i / lengths.size() * (vertices.size() - 1);
}

float pathing::PolygonPath::arc_parameter(float t) const {
    return lengths[(int) (t / (vertices.size() - 1) * lengths.size())];
}

void pathing::PolygonPath::compute(const Eigen::VectorXf& t, Eigen::Matrix<float, 2, -1>& res, int deriv) const {
    for (int i = 0; i < t.size(); i++) {
        res.col(i) = compute(t(i), deriv);
    }
}

Eigen::Matrix<float, 2, -1> pathing::PolygonPath::compute(const Eigen::VectorXf& t, int deriv) const {
    Eigen::Matrix<float, 2, -1> x;
    x.resize(2, t.size());
    compute(t, x, deriv);
    return x;
}

void pathing::PolygonPath::compute(float t, Eigen::Vector2f& res, int deriv) const {
    int i = (int) t - (int) (t == vertices.size() - 1); t = t - i;
    Eigen::Vector2f p1 = vertices[i];
    Eigen::Vector2f p2 = vertices[(i + 1)];
    res = p1 + (p2 - p1) * t;
}

Eigen::Vector2f pathing::PolygonPath::compute(float t, int deriv) const {
    int i = (int) t - (int) (t == vertices.size() - 1); t = t - i;
    Eigen::Vector2f p1 = vertices[i];
    Eigen::Vector2f p2 = vertices[(i + 1)];
    return p1 + (p2 - p1) * t;
}

Eigen::Vector2f pathing::PolygonPath::normal(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return Eigen::Vector2f(-d(1), d(0));
}

float pathing::PolygonPath::angle(float t) const {
    Eigen::Vector2f d = compute(t, 1);
    return atan2(d(1), d(0));
}