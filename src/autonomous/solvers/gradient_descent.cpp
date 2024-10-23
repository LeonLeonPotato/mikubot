#include "autonomous/solvers/gradient_descent.h"

using namespace solvers;

float solvers::gradient_descent_single(
    func_t deriv,
    float guess, float start_bound, float end_bound, 
    float step_size, int iterations
) {
    while (iterations--) {
        float den = deriv(guess);
        float new_guess = std::fmaf(-step_size, den, guess);

        guess = std::clamp(new_guess, start_bound, end_bound);
    }

    return guess;
}

std::pair<float, float> solvers::gradient_descent_vec(
    func_vec_t func, func_vec_t deriv,
    Eigen::VectorXf guess, float start_bound, float end_bound, 
    float step_size, int iterations
) {
    while (iterations--) {
        Eigen::VectorXf den = deriv(guess).array() * step_size;

        guess.noalias() -= den;
        guess.noalias() = guess.cwiseMax(start_bound);
        guess.noalias() = guess.cwiseMin(end_bound);
    }

    Eigen::VectorXf f_guess = deriv(guess);

    float max_guess = -1;
    float max_key = -1;
    for (int i = 0; i < guess.size(); i++) {
        if (f_guess(i) < max_key) {
            max_guess = guess(i);
            max_key = f_guess(i);
        }
    }

    return {max_guess, max_key};
}