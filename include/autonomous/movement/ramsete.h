#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"
#include "autonomous/future.h"

namespace movement {

struct RamseteParamsPOD {
    float beta, zeta;
    bool use_tropical_solver = true;
};

struct RamseteParams : public MovementParams, public RamseteParamsPOD { };

class Ramsete : public BaseMovement {
    private:
        const RamseteParams& get_global_params() const override {
            return params;
        }

        TickResult tick(
            pathing::BasePath& path, const MovementParams& params, PIDGroup pids, 
            const solvers::FunctionGroup& funcs, float t
        ) const;

    public:
        RamseteParams params;

        Ramsete(
            float beta, float zeta,
            bool use_tropical_solver = true,
            std::optional<path_solver_t> initializer = solve_path_default, 
            std::optional<solvers::Solver> solver_override = solvers::Solver::None
        ) : BaseMovement(initializer, solver_override) { 
            params.beta = beta;
            params.zeta = zeta;
            params.use_tropical_solver = use_tropical_solver;
        }

        MovementResult follow_path_cancellable(
            volatile bool& cancel_ref, 
            pathing::BasePath& path,
            const MovementParams& params,
            PIDGroup pids
        ) const override;
};

} // namespace movement