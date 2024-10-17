#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

namespace pathing {
namespace variables {
    float turning_coeff = 80.8507;
    float distance_coeff = 5.0;
};

void goto_pos_tick(const Eigen::Vector2f& point) {
    float theta_diff = robot::angular_diff(point);
    float dist = robot::distance(point);
    float dist_vel = fmin(dist * variables::distance_coeff, 127);
    robot::velo(
        dist_vel + theta_diff * variables::turning_coeff, 
        dist_vel - theta_diff * variables::turning_coeff
    );
}

void turn_towards_tick(float angle) {
    float theta_diff = robot::angular_diff(angle);
    robot::volt(
        theta_diff * variables::turning_coeff, 
        -theta_diff * variables::turning_coeff
    );
}

void goto_pos(const Eigen::Vector2f& point, float threshold) {
    while (robot::distance(point) > threshold) {
        goto_pos_tick(point);
        pros::delay(20);
    }
}

void turn_towards(float angle, float threshold) {
    while (fabs(robot::angular_diff(angle)) > threshold) {
        turn_towards_tick(angle);
        pros::delay(20);
    }
}

namespace pure_pursuit {
std::pair<float, float> compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, float radius,
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
    return {guess, dist};
}

std::pair<float, float> compute_intersections(spline::AbstractSpline& spline, Eigen::Vector2f& point, float radius,
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
} // namespace pure_pursuit
} // namespace pathing