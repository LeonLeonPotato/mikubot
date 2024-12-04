#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"
#include "autonomous/future.h"

namespace movement::purepursuit {
struct PurePursuitParamsPOD {
    int update_iterations = 10;
    float update_threshold = 1e-1;

    NumericalRecomputation force_recomputation = NumericalRecomputation::NONE;
    solvers::Solver solver = solvers::Solver::Newton;
    int recomputation_guesses = 16;
    float recomputation_threshold = 1.5;
    int recomputation_iterations = 12;

    float radius = 30;
};

struct PurePursuitParams 
    : public SimpleMovementParams, public PurePursuitParamsPOD { };

struct PurePursuitResult : public SimpleResult {
    float t = 0;
    int num_path_recomputations = 0;
    int num_time_recomputations = 0;
    NumericalRecomputation recomputation_level = NumericalRecomputation::NONE;
};

PurePursuitResult follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const PurePursuitParams& params,
    const PIDGroup pids,
    path_solver_t path_solver = utils::solve_path_default
);

PurePursuitResult follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const float radius,
    const SimpleMovementParams& params,
    const PIDGroup pids,
    path_solver_t path_solver = utils::solve_path_default
);

template <typename... Args>
PurePursuitResult follow_path(Args&&... args) {
    const bool cancel = false;
    return follow_path_cancellable((volatile bool&) cancel, std::forward<Args>(args)...);
}

template <typename... Args>
Future<PurePursuitResult> follow_path_async(Args&&... args) {
    Future<PurePursuitResult> ret;
    pros::Task task {[&ret, &args...]() {
        ret.set_value(std::move(
            follow_path_cancellable(ret.get_state()->cancelled, std::forward<Args>(args)...)
        ));
    }};
    return ret;
}
} // namespace movement::purepursuit