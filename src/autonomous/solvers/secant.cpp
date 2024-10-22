#include "autonomous/solvers/secant.h"

using namespace solvers;

std::pair<float, float> solvers::secant_single(
    func_t func,
    float t0, float t1, float start_bound, float end_bound, int iterations, float threshold
) {
    while (iterations--) {
        float ft0 = func(t0);
        float ft1 = func(t1);

        if (fabs(ft1) < threshold) break;

        float t2 = t1 - ft1 * (t1 - t0) / (ft1 - ft0);
        t2 = std::clamp(t2, start_bound, end_bound);
        t0 = t1; t1 = t2;
    }

    float ft1 = func(t1);

    return {t1, ft1};
}

#include <iostream>
std::pair<float, float> solvers::secant_vec(
    func_vec_t func,
    Eigen::VectorXf t0, Eigen::VectorXf t1, float start_bound, float end_bound, int iterations, float threshold
) {
    while (iterations--) {
        Eigen::VectorXf ft0 = func(t0);
        Eigen::VectorXf ft1 = func(t1);
        Eigen::VectorXf diff = ft1 - ft0;
        diff = diff.array() + (diff.array() == 0).cast<float>() * 1e-6;

        Eigen::VectorXf t2 = t1 - ft1.cwiseProduct(t1 - t0).cwiseQuotient(diff);
        t2 = t2.cwiseMax(start_bound).cwiseMin(end_bound);
        t0 = t1; t1 = t2;
    }

    Eigen::VectorXf ft1 = func(t1);

    float max_guess = -1;
    float max_key = -1;
    for (int i = 0; i < t1.size(); i++) {
        if (ft1(i) < threshold && t1(i) > max_guess) {
            max_guess = t1(i);
            max_key = ft1(i);
        }
    }

    return {max_guess, max_key};
}