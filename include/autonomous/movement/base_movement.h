#pragma once

#include "autonomous/controllers.h"
#include "autonomous/pathing/base_path.h"

#include "Eigen/Dense"

namespace movement {
using path_solver_t = std::function<void(pathing::BasePath& path)>;

enum class ExitCode {
    TBD = -1,
    SUCCESS = 0,
    RECOMPUTATION_ERROR = 1,
    TIMEOUT = 2,
    CANCELLED = 3
};

enum class NumericalRecomputation {
    NONE = 0,
    TIME = 1,
    PATH_AND_TIME = 2
};

struct NumericalRecomputationParams {
    int iterations = 10;
    float threshold = 1e-1;
    float step_size = 0.1;

    const solvers::FunctionGroup& funcs;
    const solvers::Solver solver;
};

struct SimpleResult {
    ExitCode code = ExitCode::TBD;
    float error = 0;
    int time_taken_ms = 0;
};

struct SimpleMovementParams {
    bool reversed = false;
    float exit_threshold = 5.0;

    float max_linear_speed = 1.0;
    bool use_cosine_scaling = true;

    int timeout = 2000;
    int delay = 20;

    void copy_from(const SimpleMovementParams& other) {
        reversed = other.reversed;
        exit_threshold = other.exit_threshold;
        max_linear_speed = other.max_linear_speed;
        use_cosine_scaling = other.use_cosine_scaling;
        timeout = other.timeout;
        delay = other.delay;
    }
};

struct PIDGroup {
    controllers::PID& angular;
    controllers::PID& linear;

    void reset(void) const {
        angular.reset();
        linear.reset();
    }
};
} // namespace movement

namespace movement::utils {
std::pair<float, float> compute_initial_t(
    const pathing::BasePath& path, 
    const float guesses,
    const NumericalRecomputationParams& params);

std::pair<float, float> compute_updated_t(
    pathing::BasePath& path, 
    const float t,
    const NumericalRecomputationParams& params);

void recompute_path(
    pathing::BasePath& path, 
    const path_solver_t path_solver,    
    const int goal_i);

void solve_path_default(pathing::BasePath& path);
}