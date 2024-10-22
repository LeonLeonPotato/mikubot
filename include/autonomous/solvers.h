#pragma once

#include "autonomous/solvers/bisect.h"
#include "autonomous/solvers/newton.h"
#include "autonomous/solvers/secant.h"

namespace solvers {
enum class Solver {
    Bisection,
    Newton,
    Secant
};
}