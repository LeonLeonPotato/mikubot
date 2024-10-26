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

    public:
        pathing::BasePath& path;
        pathing::BaseParams& solve_params;

        controllers::PID pid;

        solvers::Solver solver_override = solvers::Solver::None;
        virtual solvers::Solver get_solver() const {
            return solver_override == solvers::Solver::None ? path.get_solver() : solver_override;
        }

        float final_threshold = 5.0;
        float distance_coeff = 5.0;

        int update_iterations = 3;
        float update_threshold = 1e-1;
        float grad_desc_step_size = 0.1;

        float recomputation_error = 1.5;
        int recomputation_iterations = 12;
        float recomputation_threshold = 1e-1;

        int timeout = 5000;

        BaseMovement(pathing::BasePath& path, pathing::BaseParams& solve_params, const controllers::PID& pid) :
            path(path), solve_params(solve_params), pid(pid) {}

        BaseMovement(pathing::BasePath& path, pathing::BaseParams& solve_params) :
            path(path), solve_params(solve_params) { set_generic_pid(); }

        solvers::func_t func() const { return func_; }
        solvers::func_t deriv() const { return deriv_; }
        solvers::func_vec_t vec_func() const { return vec_func_; }
        solvers::func_vec_t vec_deriv() const { return vec_deriv_; }

        virtual TickResult tick(float t) = 0;
        virtual MovementResult follow_path_cancellable(bool& cancel_ref) = 0;

        virtual MovementResult follow_path(void) {
            bool cancel = false;
            return follow_path_cancellable(cancel);
        }
        virtual Future<MovementResult> follow_path_async() {
            Future<MovementResult> future;
            pros::Task task([this, &future]() {
                future.set_value(follow_path_cancellable(future.get_state()->cancelled));
            });
            return future;
        }

        // for convenience
        float maxt() const { return path.points.size() - 1; }
        void set_generic_pid() { init_generic_pid(pid); }

        static void init_generic_pid(controllers::PID& pid);
};

namespace utils {
std::pair<float, float> compute_initial_t_newton(BaseMovement& mover);

std::pair<float, float> compute_initial_t_secant(BaseMovement& mover);

std::pair<float, float> compute_updated_t_newton(BaseMovement& mover, float t);

std::pair<float, float> compute_updated_t_secant(BaseMovement& mover, float t);

float compute_updated_t_grad_desc(BaseMovement& mover, float t);

std::pair<float, float> compute_initial_t(solvers::Solver solver, BaseMovement& mover);

std::pair<float, float> compute_updated_t(solvers::Solver solver, BaseMovement& mover, float t);

void recompute_path(BaseMovement& mover, int goal_i);
} // namespace utils

void init_generic_pid(controllers::PID& pid);
void goto_pos_tick(const Eigen::Vector2f& point, float distance_coeff, controllers::PID& pid);
void turn_towards_tick(float angle, controllers::PID& pid);
void goto_pos(const Eigen::Vector2f& point, float distance_coeff, float threshold);
void turn_towards(float angle, float threshold);
void goto_pose(const Eigen::Vector2f& point, float angle, float distance_coeff, float threshold, float theta_threshold);
} // namespace movement