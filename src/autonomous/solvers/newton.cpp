#include "autonomous/solvers/newton.h"

using namespace movement;

std::pair<float, float> solvers::newton(
    func_t func, func_t deriv,
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
    func_vec_t func, func_vec_t deriv,
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