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

    std::string debug_out(void) {
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
        const BaseMovementParams params;

        pathing::BasePath& path;
        std::function<void(pathing::BaseParams&)> solve_params_initializer = __initialize_solve_params;
        controllers::PID pid;

        solvers::Solver solver_override = solvers::Solver::None;

        BaseMovement(
            pathing::BasePath& path, 
            const std::function<void(pathing::BaseParams&)> initializer, 
            const BaseMovementParams& params,
            const controllers::PID& pid,
            const solvers::Solver solver_override
        ) : path(path), solve_params_initializer(initializer), params(params),
            pid(pid), solver_override(solver_override) {}

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
            pros::Task task([this, &future]() {
                auto res = follow_path_cancellable(future.get_state()->cancelled);
                future.set_value(res);
            });
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

class BaseMovementBuilder {
    std::optional<BaseMovementParams> params = std::nullopt;
    std::optional<controllers::PID> pid = std::nullopt;
    pathing::BasePath* path = nullptr; // we definitely do not want to copy path!
    std::function<void(pathing::BaseParams&)> solve_params_initializer = BaseMovement::init_generic_solve_params;
    solvers::Solver solver_override = solvers::Solver::None;

    BaseMovementBuilder& with_params(const BaseMovementParams params);
    BaseMovementBuilder& with_pid(const controllers::PID pid);
    BaseMovementBuilder& with_path(pathing::BasePath& path);
    BaseMovementBuilder& with_solve_params_initializer(const std::function<void(pathing::BaseParams&)>);
    BaseMovementBuilder& with_solver_override(const solvers::Solver solver);

    BaseMovement build(void) const;
    virtual std::shared_ptr<BaseMovement> build_no_copy(void) const;
};
} // namespace movement