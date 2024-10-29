#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"
#include "autonomous/future.h"

namespace movement {

struct PurePursuitParams : public BaseMovementParams {
    float radius;
};

class PurePursuit : public BaseMovement {
    protected:
        float func(float t) const override;
        float deriv(float t) const override;
        Eigen::VectorXf vec_func(Eigen::VectorXf& t) const override;
        Eigen::VectorXf vec_deriv(Eigen::VectorXf& t) const override;

    public:
        PurePursuit(
            pathing::BasePath& path, 
            float radius,
            std::optional<solver_init_t> initializer, 
            std::optional<const controllers::PID&> pid = std::nullopt,
            std::optional<const solvers::Solver> solver_override = std::nullopt
        ) : BaseMovement(path, PurePursuitParams(), initializer, pid, solver_override) {
            ((PurePursuitParams&) params).radius = radius;
        }

        TickResult tick(float t) override;
        MovementResult follow_path_cancellable(bool& cancel_ref) override;
};

} // namespace movement