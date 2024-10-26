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

        PurePursuit(pathing::BasePath& path, pathing::BaseParams& solve_params, const controllers::PID& pid, float radius) :
            BaseMovement(path, solve_params, pid), radius(radius) {}
        PurePursuit(pathing::BasePath& path, pathing::BaseParams& solve_params, float radius) :
            BaseMovement(path, solve_params), radius(radius) {}

        TickResult tick(float t) override;
        MovementResult follow_path_cancellable(bool& cancel_ref) override;
};

} // namespace movement