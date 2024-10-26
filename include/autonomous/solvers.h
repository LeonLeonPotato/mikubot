#pragma once

#include "autonomous/solvers/bisect.h"
#include "autonomous/solvers/newton.h"
#include "autonomous/solvers/secant.h"
#include "autonomous/solvers/gradient_descent.h"

namespace solvers {
enum class Solver {
    None,
    Bisection,
    Newton,
    Secant,
    GradientDescent
};
}