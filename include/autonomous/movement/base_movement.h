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

struct TickResult {
    ExitCode code = ExitCode::TBD;

    float t = 0;
    float error = 0;
    bool recomputed = false;
};

struct MovementResult {
    ExitCode code = ExitCode::TBD;

    int time_taken_ms = 0;
    int num_recomputations = 0;
    float t = 0;
    float error = 0;

    std::string debug_out(void) const {
        char buffer[256];
        sprintf(buffer, "MovementResult {Code: %d, Time: %d, Recomputations: %d, T: %f, Error: %f}", code, time_taken_ms, num_recomputations, t, error);
        return std::string(buffer);
    }
};

struct BaseMovementParams {
    bool reversed = false;

    float final_threshold = 5.0;
    float distance_coeff = 5.0;
    int max_base_speed = 127;

    int update_iterations = 3;
    float update_threshold = 1e-1;
    float grad_desc_step_size = 0.1;

    bool always_recompute = false;
    int recomputation_guesses = 32;
    float recomputation_error = 1.5;
    int recomputation_iterations = 12;
    float recomputation_threshold = 1e-1;

    int timeout = 5000;
};

class BaseMovement {
    protected:
        using solver_init_t = std::function<void(pathing::BaseParams&)>;

        std::pair<float, float> compute_initial_t_newton(
            const pathing::BasePath& path, const BaseMovementParams& params,
            solvers::func_vec_t func, solvers::func_vec_t deriv) const;
        std::pair<float, float> compute_initial_t_secant(
            const pathing::BasePath& path, const BaseMovementParams& params,
            solvers::func_vec_t func) const;
        std::pair<float, float> compute_updated_t_newton(
            const pathing::BasePath& path, const BaseMovementParams& params,
            solvers::func_t func, solvers::func_t deriv, float t) const;
        std::pair<float, float> compute_updated_t_secant(
            const pathing::BasePath& path, const BaseMovementParams& params,
            solvers::func_t func, float t) const;
        float compute_updated_t_grad_desc(
            const pathing::BasePath& path, const BaseMovementParams& params,
            solvers::func_t deriv, float t) const;
        
        std::pair<float, float> compute_initial_t(
            const pathing::BasePath& path, const BaseMovementParams& params, 
            std::optional<solvers::func_vec_t> func = std::nullopt,
            std::optional<solvers::func_vec_t> deriv = std::nullopt,
            solvers::Solver solver = solvers::Solver::None) const;
        std::pair<float, float> compute_updated_t(
            const pathing::BasePath& path, const BaseMovementParams& params, float t,
            std::optional<solvers::func_t> func = std::nullopt,
            std::optional<solvers::func_t> deriv = std::nullopt,
            solvers::Solver solver = solvers::Solver::None
        ) const;

        void recompute_path(pathing::BasePath& path, int goal_i) const;

        virtual solvers::Solver get_solver(const pathing::BasePath& path) const {
            return solver_override == solvers::Solver::None ? path.get_solver() : solver_override;
        }

    public:
        solver_init_t solve_params_initializer = init_generic_solve_params;
        solvers::Solver solver_override = solvers::Solver::None;

        BaseMovement(
            std::optional<solver_init_t> initializer = std::nullopt, 
            std::optional<solvers::Solver> solver_override = std::nullopt
        )
        {
            if (initializer.has_value()) this->solve_params_initializer = initializer.value();
            if (solver_override.has_value()) this->solver_override = solver_override.value();
        }

        virtual TickResult tick(pathing::BasePath& path, float t) = 0;

        MovementResult follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path);
        MovementResult follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path, const BaseMovementParams& params);
        MovementResult follow_path_cancellable(bool& cancel_ref, pathing::BasePath& path, controllers::PID& pid);
        virtual MovementResult follow_path_cancellable(
            bool& cancel_ref, 
            pathing::BasePath& path,
            const BaseMovementParams& params,
            controllers::PID& pid
        ) = 0;

        template <typename... Args>
        MovementResult follow_path(Args&&... args) {
            bool cancel = false;
            return follow_path_cancellable((bool&) cancel, std::forward<Args>(args));
        }

        template <typename... Args>
        Future<MovementResult> follow_path_async(Args&&... args) {
            Future<MovementResult> ret;
            pros::Task task {[this, &ret, args...]() {
                ret.set_value(follow_path((bool&) ret.get_state()->available, args...));
            }};
        }

        static void init_generic_pid(controllers::PID& pid);
        static void init_generic_solve_params(pathing::BaseParams& solve_params);
};
} // namespace movement