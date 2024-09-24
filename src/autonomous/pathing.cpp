#include "autonomous/pathing.h"
#include "robot.h"

#include "api.h"

inline void keysort(Eigen::VectorXf& v, Eigen::VectorXf& keys) {
    std::vector<int> indices(keys.size());
    for(int i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }

    std::sort(indices.begin(), indices.end(),
              [&keys](int i1, int i2) { return keys[i1] < keys[i2]; });

    Eigen::VectorXf sorted_v(v.size());
    Eigen::VectorXf sorted_keys(keys.size());

    for(int i = 0; i < indices.size(); ++i) {
        sorted_v[i] = v[indices[i]];
        sorted_keys[i] = keys[indices[i]];
    }

    v = sorted_v;
    keys = sorted_keys;
}

namespace pure_pursuit {
float compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, float guess,
                            float iterations, float threshold, float radius, float start_bound, float end_bound) 
{
    while (iterations--) {
        Eigen::Vector2f f_guess = spline.compute(guess) - point;
        float num = f_guess.dot(f_guess) - radius * radius;
        float den = 2 * f_guess.dot(spline.compute(guess, 1));
        guess -= num / (den + 1e-5);
        guess = std::max(std::min(guess, end_bound), start_bound);
    }

    float dist = (spline.compute(guess) - point).norm() - radius;
    if (dist > threshold) return -1;
    return guess;
}

float compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, Eigen::VectorXf& guess,
                            float iterations, float threshold, float radius, float start_bound, float end_bound) 
{
    while (iterations--) {
        Eigen::Matrix2Xf f_guess = spline.compute(guess) - point;
        Eigen::VectorXf num = f_guess.cwiseProduct(f_guess).colwise().sum().array() - radius * radius;
        Eigen::VectorXf den = 2 * f_guess.cwiseProduct(spline.compute(guess, 1)).colwise().sum().array() + 1e-5;
        guess -= num.cwiseQuotient(den);
        guess = guess.cwiseMax(start_bound).cwiseMin(end_bound);
    }

    Eigen::Matrix2Xf f_guess = spline.compute(guess) - point;
    Eigen::VectorXf keys = f_guess.cwiseProduct(f_guess).colwise().sum().sqrt().array() - radius;

    keysort(guess, keys);

    for (int i = keys.size()-1; i >= 0; i--) {
        if (keys[i] < threshold) return guess[i];
    }

    return -1;
}
} // namespace pathing