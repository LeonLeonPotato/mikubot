#pragma once

#include "autonomous/controllers.h"
#include "autonomous/pathing/base_path.h"

#include "Eigen/Dense"

#define deg(x) (x * M_PI / 180.0)
#define __timediff(x) ((int) (pros::millis() - x))

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

    std::string debug_out(void) {
        return "NumericalRecomputationParams { iterations: " 
            + std::to_string(iterations) + ", threshold: " 
            + std::to_string(threshold) + ", step_size: " 
            + std::to_string(step_size) + " }";
    }
};

struct SimpleResult {
    ExitCode code = ExitCode::TBD;
    float error = 0;
    int time_taken_ms = 0;

    std::string debug_out(void) {
        return "SimpleResult { code: " 
            + std::to_string((int) code) + ", error: " 
            + std::to_string(error) + ", time: " 
            + std::to_string(time_taken_ms) + " }";
    }
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

    std::string debug_out(void) {
        return "SimpleMovementParams { reversed: "
            + std::to_string(reversed) + ", exit_threshold: "
            + std::to_string(exit_threshold) + ", max_linear_speed: "
            + std::to_string(max_linear_speed) + ", use_cosine_scaling: " 
            + std::to_string(use_cosine_scaling) + ", timeout: " 
            + std::to_string(timeout) + ", delay: " 
            + std::to_string(delay) + " }";
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