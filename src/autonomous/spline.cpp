#include "autonomous/spline.h"
#include "api.h"

namespace spline {
Eigen::Matrix<float, 6, 6> differential_matrix_1;
Eigen::Matrix<float, 6, 6> differential_matrix_0;

void init(void) {
    differential_matrix_1 <<
        1, 1, 1, 1, 1, 1,
        0, 1, 2, 3, 4, 5,
        0, 0, 2, 6, 12, 20,
        0, 0, 0, 6, 24, 60,
        0, 0, 0, 0, 24, 120,
        0, 0, 0, 0, 0, 120;

    differential_matrix_0 <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        0, 0, 0, 6, 0, 0,
        0, 0, 0, 0, 24, 0,
        0, 0, 0, 0, 0, 120;
}

void Polynomial(void) {
    Polynomial
}
}