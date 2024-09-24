#include "autonomous/pathing.h"
#include "robot.h"

#include "api.h"

namespace pure_pursuit {
float compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, float radius,
                            float guess, float start_bound, float end_bound, int iterations, float threshold) 
{
    while (iterations--) {
        Eigen::Vector2f rel = spline.compute(guess) - point;
        float num = rel.dot(rel) - radius * radius;
        Eigen::Vector2f deriv = spline.compute(guess, 1);
        float den = 2 * rel.dot(deriv);

        if (fabs(den) < 1e-5) break;

        float new_guess = guess - num / den;
        guess = std::max(std::min(new_guess, end_bound), start_bound);

        if (fabs(new_guess - guess) < threshold) break;
    }

    float dist = fabs((spline.compute(guess) - point).norm() - radius);
    if (dist > threshold) return -1;
    return guess;
}

float compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, float radius,
                            Eigen::VectorXf guess, float start_bound, float end_bound, int iterations, float threshold) 
{
    while (iterations--) {
        Eigen::Matrix2Xf rel = spline.compute(guess).colwise() - point;
        Eigen::VectorXf num = rel.cwiseProduct(rel).colwise().sum().array() - radius * radius;
        Eigen::Matrix2Xf deriv = spline.compute(guess, 1);
        Eigen::VectorXf den = 2 * rel.cwiseProduct(deriv).colwise().sum().array() + 1e-5;
        guess -= num.cwiseQuotient(den);
        guess = guess.cwiseMax(start_bound).cwiseMin(end_bound);
    }

    Eigen::Matrix2Xf f_guess = spline.compute(guess).colwise() - point;
    Eigen::VectorXf keys = (f_guess.cwiseProduct(f_guess).colwise().sum().cwiseSqrt().array() - radius).cwiseAbs();

    float max_guess = -1;
    for (int i = 0; i < guess.size(); i++) {
        if (keys(i) < threshold) {
            max_guess = std::max(max_guess, guess(i));
        }
    }

    return max_guess;
}
} // namespace pathing