#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"
#include "autonomous/future.h"

namespace movement {

class PurePursuit : public BaseMovement {
    private:
        float func(pathing::BasePath& path, float t) const;
        float deriv(pathing::BasePath& path, float t) const;
        Eigen::VectorXf vec_func(pathing::BasePath& path, Eigen::VectorXf& t) const;
        Eigen::VectorXf vec_deriv(pathing::BasePath& path, Eigen::VectorXf& t) const;

    TickResult tick(
        pathing::BasePath& path, const MovementParams& params, controllers::PID& pid, 
        const solvers::FunctionGroup& funcs, float t
    ) const override;

    public:
        float radius;

        PurePursuit(
            float radius,
            std::optional<path_solver_t> initializer = solve_path_default, 
            std::optional<solvers::Solver> solver_override = solvers::Solver::None
        ) : BaseMovement(initializer, solver_override), radius(radius) { }

        MovementResult follow_path_cancellable(
            bool& cancel_ref, 
            pathing::BasePath& path,
            const MovementParams& params,
            controllers::PID& pid
        ) const override;
};

} // namespace movement