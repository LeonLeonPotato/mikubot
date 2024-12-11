#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"
#include "autonomous/future.h"

namespace movement::ramsete {
struct RamseteParamsPOD {
    float beta, zeta;
    bool use_tropical_solver = true;
};

struct RamseteParams 
    : public SimpleMovementParams, public RamseteParamsPOD { };

struct RamseteResult : public SimpleResult {
    int i = 1;
};

RamseteResult follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const RamseteParams& params
);

RamseteResult follow_path_cancellable(
    volatile bool& cancel_ref, 
    pathing::BasePath& path,
    const float beta, const float zeta,
    const SimpleMovementParams& params
);

template <typename... Args>
RamseteResult follow_path(Args&&... args) {
    const bool cancel = false;
    return follow_path_cancellable((volatile bool&) cancel, std::forward<Args>(args)...);
}

template <typename... Args>
Future<RamseteResult> follow_path_async(Args&&... args) {
    Future<RamseteResult> ret;
    pros::Task task {[&ret, &args...]() {
        ret.set_value(std::move(
            follow_path_cancellable(ret.get_state()->cancelled, std::forward<Args>(args)...)
        ));
    }};
    return ret;
}
} // namespace movement::ramsete