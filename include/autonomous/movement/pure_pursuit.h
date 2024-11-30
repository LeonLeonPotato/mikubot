#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"
#include "autonomous/future.h"

namespace movement {

struct PurePursuitParamsPOD {
    float radius = 30;
};

struct PurePursuitParams : public MovementParams, public PurePursuitParamsPOD { };

class PurePursuit : public BaseMovement {
    private:
        float func(pathing::BasePath& path, float radius, float t) const;
        float deriv(pathing::BasePath& path, float t) const;
        Eigen::VectorXf vec_func(pathing::BasePath& path, float radius, Eigen::VectorXf& t) const;
        Eigen::VectorXf vec_deriv(pathing::BasePath& path, Eigen::VectorXf& t) const;

        const PurePursuitParams& get_global_params() const override {
            return params;
        }

        TickResult tick(
            pathing::BasePath& path, const MovementParams& params, PIDGroup pids, 
            const solvers::FunctionGroup& funcs, float t
        ) const override;

    public:
        PurePursuitParams params;

        PurePursuit(
            float radius = 30,
            std::optional<path_solver_t> initializer = solve_path_default, 
            std::optional<solvers::Solver> solver_override = solvers::Solver::None
        ) : BaseMovement(initializer, solver_override) { 
            params.radius = radius;
        }

        MovementResult follow_path_cancellable(
            volatile bool& cancel_ref, 
            pathing::BasePath& path,
            const MovementParams& params,
            PIDGroup pids
        ) const override;
};

} // namespace movement