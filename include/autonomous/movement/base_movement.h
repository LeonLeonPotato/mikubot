#pragma once

#include "autonomous/future.h"
#include "autonomous/controllers.h"
#include "autonomous/pathing/base_path.h"

#include "Eigen/Dense"

namespace movement {
enum class ExitCode {
    TBD = -1,
    SUCCESS = 0,
    RECOMPUTATION_ERROR = 1,
    TIMEOUT = 2,
    CANCELLED = 3
};

enum class RecomputationLevel {
    NONE = 0,
    TIME = 1,
    PATH_AND_TIME = 2
};

struct TickResult {
    ExitCode code = ExitCode::TBD;

    float t = 0;
    float error = 0;
    RecomputationLevel recomputation_level = RecomputationLevel::NONE;
};

struct MovementResult {
    ExitCode code = ExitCode::TBD;

    int time_taken_ms = 0;
    int num_path_recomputations = 0;
    int num_time_recomputations = 0;
    float t = 0;
    float error = 0;

    std::string debug_out(void) const {
        char buffer[256];
        sprintf(buffer, "MovementResult {Code: %d, Time taken: %d, Path recomputations: %d, Time recomputations: %d, t parameter: %f, Error: %f}", 
            code, time_taken_ms, num_path_recomputations, num_time_recomputations, t, error);
        return std::string(buffer);
    }
};

struct MovementParams {
    bool reversed = false;

    float final_threshold = 5.0;
    float distance_coeff = 5.0;
    int max_base_speed = 127;

    int update_iterations = 3;
    float update_threshold = 1e-1;
    float grad_desc_step_size = 0.1;

    RecomputationLevel force_recomputation = RecomputationLevel::NONE;
    int recomputation_guesses = 32;
    float recomputation_threshold = 1.5;
    int recomputation_iterations = 12;

    int timeout = 5000;
    int delay = 20;
};

class BaseMovement {
    protected:
        using path_solver_t = std::function<void(pathing::BasePath& path)>;

        std::pair<float, float> compute_initial_t(
            const pathing::BasePath& path, const MovementParams& params,
            const solvers::FunctionGroup& funcs, 
            solvers::Solver solver = solvers::Solver::None) const;

        std::pair<float, float> compute_updated_t(
            pathing::BasePath& path, const MovementParams& params,
            const solvers::FunctionGroup& funcs, float t, 
            solvers::Solver solver = solvers::Solver::None) const;

        void recompute_path(pathing::BasePath& path, int goal_i) const;

        virtual solvers::Solver get_solver(const pathing::BasePath& path) const {
            return solver_override == solvers::Solver::None ? path.get_solver() : solver_override;
        }

        virtual TickResult tick(
            pathing::BasePath& path, const MovementParams& params, controllers::PID& pid, 
            const solvers::FunctionGroup& funcs, float t
        ) const = 0;

    public:
        path_solver_t path_solver;
        solvers::Solver solver_override;
        MovementParams default_params;

        BaseMovement(
            std::optional<path_solver_t> path_solver = solve_path_default, 
            std::optional<solvers::Solver> solver_override = solvers::Solver::None
        ) : path_solver(path_solver.value()), solver_override(solver_override.value()) { }

        MovementResult follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path) const;
        MovementResult follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path, const MovementParams& params) const;
        MovementResult follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path, controllers::PID& pid) const;
        virtual MovementResult follow_path_cancellable(
            bool& cancel_ref, 
            pathing::BasePath& path,
            const MovementParams& params,
            controllers::PID& pid
        ) const = 0;

        template <typename... Args>
        MovementResult follow_path(Args&&... args) const {
            bool cancel = false;
            return follow_path_cancellable((bool&) cancel, std::forward<Args>(args)...);
        }

        template <typename... Args>
        Future<MovementResult> follow_path_async(Args&&... args) const {
            Future<MovementResult> ret;
            pros::Task task {[this, &ret, &args...]() {
                ret.set_value(std::move(follow_path_cancellable((bool&) ret.get_state()->available, std::forward<Args>(args)...)));
            }};
            return ret;
        }

        static void init_generic_pid(controllers::PID& pid);
        static void solve_path_default(pathing::BasePath& path);
};
} // namespace movement