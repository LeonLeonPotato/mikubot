#include "autonomous/movement/numerical_solvers.h"

using namespace movement;

std::pair<float, float> solvers::newton(
    func_t func, deriv_t deriv,
    float guess, float start_bound, float end_bound, int iterations, float threshold
) {
    float dist;
    while (iterations--) {
        float num = func(guess);
        float den = deriv(guess);

        dist = fabs(num);
        if (dist < threshold) break;

        float new_guess = guess - num / den;
        guess = std::clamp(new_guess, start_bound, end_bound);
    }

    return {guess, dist};
}

std::pair<float, float> solvers::newton(
    func_vec_t func, deriv_vec_t deriv,
    Eigen::VectorXf guess, float start_bound, float end_bound, int iterations, float threshold
) {
    while (iterations--) {
        Eigen::VectorXf num = func(guess);
        Eigen::VectorXf den = deriv(guess);
        guess -= num.cwiseQuotient(den);
        guess = guess.cwiseMax(start_bound).cwiseMin(end_bound);
    }

    Eigen::VectorXf f_guess = func(guess);

    float max_guess = -1;
    float max_key = -1;
    for (int i = 0; i < guess.size(); i++) {
        if (f_guess(i) < threshold) {
            max_guess = std::max(max_guess, guess(i));
            max_key = f_guess(i);
        }
    }

    return {max_guess, max_key};
}

std::pair<float, float> solvers::secant(
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

std::pair<float, float> solvers::secant(
    func_vec_t func,
    Eigen::VectorXf t0, Eigen::VectorXf t1, float start_bound, float end_bound, int iterations, float threshold
) {
    while (iterations--) {
        Eigen::VectorXf ft0 = func(t0);
        Eigen::VectorXf ft1 = func(t1);

        Eigen::VectorXf t2 = t1 - ft1.cwiseProduct(t1 - t0).cwiseQuotient(ft1 - ft0);
        t2 = t2.cwiseMax(start_bound).cwiseMin(end_bound);
        t0 = t1; t1 = t2;
    }

    Eigen::VectorXf ft1 = func(t1);

    float max_guess = -1;
    float max_key = -1;
    for (int i = 0; i < t1.size(); i++) {
        if (ft1(i) < threshold) {
            max_guess = std::max(max_guess, t1(i));
            max_key = ft1(i);
        }
    }

    return {max_guess, max_key};
}