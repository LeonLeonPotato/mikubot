#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"
#include "autonomous/future.h"

namespace movement {

class PurePursuit : public BaseMovement {
    protected:
        float func(float t) const override;
        float deriv(float t) const override;
        Eigen::VectorXf vec_func(Eigen::VectorXf& t) const override;
        Eigen::VectorXf vec_deriv(Eigen::VectorXf& t) const override;

    public:
        float radius;

        PurePursuit(
            pathing::BasePath& path, 
            float radius,
            std::optional<BaseMovementParams> params = std::nullopt, 
            std::optional<solver_init_t> initializer = std::nullopt, 
            std::optional<controllers::PID> pid = std::nullopt,
            std::optional<solvers::Solver> solver_override = std::nullopt
        ) : BaseMovement(path, params, initializer, pid, solver_override), radius(radius)
        { }

        TickResult tick(float t) override;
        MovementResult follow_path_cancellable(bool& cancel_ref) override;
};

} // namespace movement