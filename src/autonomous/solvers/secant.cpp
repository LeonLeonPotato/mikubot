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

        float diff = ft1 - ft0;
        diff += (diff == 0) * 1e-6;

        float t2 = t1 - ft1 * (t1 - t0) / diff;
        // printf("iterations: %d, t0: %f, t1: %f, ft0: %f, ft1: %f, t2: %f\n", iterations, t0, t1, ft0, ft1, t2);
        t2 = std::clamp(t2, start_bound, end_bound);
        t0 = t1; t1 = t2;
    }

    float ft1 = func(t1);

    return {t1, ft1};
}

std::pair<float, float> solvers::secant_single(
    const FunctionGroup& funcs,
    float t0, float t1, float start_bound, float end_bound, int iterations, float threshold
) {
    return secant_single(funcs.funcs[0], t0, t1, start_bound, end_bound, iterations, threshold);
}

std::pair<float, float> solvers::secant_vec(
    func_vec_t func,
    Eigen::VectorXf t0, Eigen::VectorXf t1, float start_bound, float end_bound, int iterations, float threshold
) {
    while (iterations--) {
        Eigen::VectorXf ft0 = func(t0);
        Eigen::VectorXf ft1 = func(t1);
        Eigen::VectorXf diff = ft1 - ft0;
        diff.array() += (diff.array() == 0).cast<float>() * 1e-6;

        Eigen::VectorXf t2 = (t1 - ft1.cwiseProduct(t1 - t0).cwiseQuotient(diff))
            .cwiseMax(start_bound).cwiseMin(end_bound);
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

std::pair<float, float> solvers::secant_vec(
    const FunctionGroup& funcs,
    const Eigen::VectorXf& t0, const Eigen::VectorXf& t1, 
    float start_bound, float end_bound, int iterations, float threshold
) {
    return secant_vec(funcs.vec_funcs[0], t0, t1, start_bound, end_bound, iterations, threshold);
}

