#include "autonomous/pathing/utils.h"

using namespace pathing;

static Eigen::MatrixXi differential_matrix;
static Eigen::MatrixXi pascal_matrix;

const int utils::falling_factorial(int i, int j) {
    if (i < j) return 0;
    int result = 1;
    for (int k = i; k > i - j; k--) result *= k;
    return result;
}

const Eigen::MatrixXi& utils::get_differential_matrix(int n) {
    if (differential_matrix.rows() <= n) {
        differential_matrix.resize(n + 1, n + 1);
        for (int i = 0; i <= n; i++) {
            for (int j = 0; j <= i; j++) {
                differential_matrix(i, j) = falling_factorial(i, j);
                differential_matrix(j, i) = falling_factorial(j, i);
            }
        }
    }
    return differential_matrix;
}

const Eigen::MatrixXi& utils::get_pascal_matrix(int n) {
    if (pascal_matrix.rows() <= n) {
        pascal_matrix.resize(n + 1, n + 1);
        pascal_matrix.row(0).setConstant(1);
        pascal_matrix.col(0).setConstant(1);
        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= n - i; j++) {
                pascal_matrix(i, j) = pascal_matrix(i-1, j) + pascal_matrix(i, j-1);
            }
        }
    }
    return pascal_matrix;
}

void utils::destroy_differential_matrix() {
    differential_matrix.resize(0, 0);
}

void utils::destroy_pascal_matrix() {
    pascal_matrix.resize(0, 0);
}