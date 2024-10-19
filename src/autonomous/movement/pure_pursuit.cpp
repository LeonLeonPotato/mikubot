#include "autonomous/movement/pure_pursuit.h"
#include "autonomous/movement/base_movement.h"
#include "essential.h"

using namespace movement::pure_pursuit;

std::pair<float, float> compute_intersections(pathing::BasePath& path, const Eigen::Vector2f& point, float radius,
                            float guess, float start_bound, float end_bound, int iterations, float threshold) 
{
    float dist;
    while (iterations--) {
        Eigen::Vector2f rel = path.compute(guess) - point;
        float num = rel.dot(rel) - radius * radius;
        Eigen::Vector2f deriv = path.compute(guess, 1);
        float den = 2 * rel.dot(deriv);

        if (fabs(den) < 1e-5) break;
        dist = fabs((rel).norm() - radius);
        if (dist < threshold) break;

        float new_guess = guess - num / den;
        guess = std::max(std::min(new_guess, end_bound), start_bound);
    }

    return {guess, dist};
}

std::pair<float, float> compute_intersections(pathing::BasePath& path, const Eigen::Vector2f& point, float radius,
                            Eigen::VectorXf guess, float start_bound, float end_bound, int iterations, float threshold) 
{
    while (iterations--) {
        Eigen::Matrix2Xf rel = path.compute(guess).colwise() - point;
        Eigen::VectorXf num = rel.cwiseProduct(rel).colwise().sum().array() - radius * radius;
        Eigen::Matrix2Xf deriv = path.compute(guess, 1);
        Eigen::VectorXf den = 2 * rel.cwiseProduct(deriv).colwise().sum().array() + 1e-5;
        guess -= num.cwiseQuotient(den);
        guess = guess.cwiseMax(start_bound).cwiseMin(end_bound);
    }

    Eigen::Matrix2Xf f_guess = path.compute(guess).colwise() - point;
    Eigen::VectorXf keys = (f_guess.cwiseProduct(f_guess).colwise().sum().cwiseSqrt().array() - radius).cwiseAbs();

    float max_guess = -1;
    float max_key = -1;
    for (int i = 0; i < guess.size(); i++) {
        if (keys(i) < threshold) {
            max_guess = std::max(max_guess, guess(i));
            max_key = keys(i);
        }
    }

    return {max_guess, max_key};
}

float follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, float radius, int iterations) {
    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    auto intersect = compute_intersections(path, point, radius, t, 0, path.points.size() - 1, iterations);
    t = intersect.first;

    Eigen::Vector2f res = path.compute(t);
    float dtheta = robot::angular_diff(res);
    float dist = robot::distance(res);
    dist = fmin(dist * 5, radius) / radius * 127;

    int left = (int) (dist + movement::variables::distance_coeff * dtheta);
    int right = (int) (dist - movement::variables::distance_coeff * dtheta);

    robot::volt(left, right);
    return t;
}