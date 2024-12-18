#include "autonomous/pathing/polygon_path.h"

using namespace pathing;

void PolygonPath::compute(float t, Eigen::Vector2f& res, int deriv) const {
    t = std::clamp(t, 0.0f, (float) maxt());
    int i = (int) t - (int) (t == points.size() - 1); t = t - i;
    if (deriv == 0)
        res = points[i] + (points[i + 1] - points[i]) * t;
    else if (deriv == 1)
        res = points[i + 1] - points[i];
    else
        res = Eigen::Vector2f(0, 0);
}