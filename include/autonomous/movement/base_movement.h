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

        const solvers::func_t func_ = [this](float t) -> float { return func(t); };
        const solvers::func_t deriv_ = [this](float t) -> float { return deriv(t); };
        const solvers::func_vec_t vec_func_ = [this](Eigen::VectorXf& t) -> Eigen::VectorXf { return vec_func(t); };
        const solvers::func_vec_t vec_deriv_ = [this](Eigen::VectorXf& t) -> Eigen::VectorXf { return vec_deriv(t); };

        virtual float func(float t) const { return -1; };
        virtual float deriv(float t) const { return -1; };
        virtual Eigen::VectorXf vec_func(Eigen::VectorXf& t) const { return Eigen::VectorXf(); };
        virtual Eigen::VectorXf vec_deriv(Eigen::VectorXf& t) const { return Eigen::VectorXf(); };

        std::pair<float, float> compute_initial_t_newton(void);
        std::pair<float, float> compute_initial_t_secant(void);
        std::pair<float, float> compute_updated_t_newton(float t);
        std::pair<float, float> compute_updated_t_secant(float t);
        float compute_updated_t_grad_desc(float t);
        
        std::pair<float, float> compute_initial_t(solvers::Solver solver);
        std::pair<float, float> compute_updated_t(solvers::Solver solver, float t);

        void recompute_path(int goal_i);

    public:
        BaseMovementParams params;

        pathing::BasePath& path;
        solver_init_t solve_params_initializer = init_generic_solve_params;
        controllers::PID pid;

        solvers::Solver solver_override = solvers::Solver::None;

        BaseMovement(
            pathing::BasePath& path, 
            std::optional<BaseMovementParams> params = std::nullopt,
            std::optional<solver_init_t> initializer = std::nullopt, 
            std::optional<controllers::PID> pid = std::nullopt,
            std::optional<solvers::Solver> solver_override = std::nullopt
        ) : path(path)
        {
            if (params.has_value()) this->params = params.value();
            if (initializer.has_value()) this->solve_params_initializer = std::move(initializer.value());
            if (pid.has_value()) this->pid = std::move(pid.value());
            if (solver_override.has_value()) this->solver_override = std::move(solver_override.value());
        }

        solvers::func_t func() const { return func_; }
        solvers::func_t deriv() const { return deriv_; }
        solvers::func_vec_t vec_func() const { return vec_func_; }
        solvers::func_vec_t vec_deriv() const { return vec_deriv_; }

        virtual TickResult tick(float t) = 0;
        virtual MovementResult follow_path_cancellable(bool& cancel_ref) = 0;

        MovementResult follow_path(void) {
            bool cancel = false;
            return follow_path_cancellable(cancel);
        }
        Future<MovementResult> follow_path_async() {
            Future<MovementResult> future;
            pros::Task task{[this, &future] {
                auto copy = future;
                copy.set_value(follow_path_cancellable(copy.get_state()->cancelled));
            }};
            return future;
        }

        virtual solvers::Solver get_solver() const {
            return solver_override == solvers::Solver::None ? path.get_solver() : solver_override;
        }

        // for convenience
        float maxt() const { return path.points.size() - 1; }
        void set_generic_pid() { init_generic_pid(pid); }

        static void init_generic_pid(controllers::PID& pid);
        static void init_generic_solve_params(pathing::BaseParams& solve_params);
};
} // namespace movement