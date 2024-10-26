#pragma once

#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"

namespace movement {
namespace stanley {

namespace variables {
    extern float step_size;

    extern float kP;
    extern float kI;
    extern float kD;

    extern float I_disable_min;
    extern float I_disable_max;
    extern float I_max;
    extern float I_min;
} // namespace variables

float follow_path_tick(pathing::BasePath& path, 
                        controllers::PID& turn_pid, controllers::PID& track_pid, 
                        solvers::func_t deriv, float t,
                        int iterations = 5);

float follow_path(pathing::BasePath& path, pathing::BaseParams& solve_params,
                controllers::PID* turn_pid = nullptr,
                controllers::PID* track = nullptr,
                int iterations = 5, int timeout = 5000);

} // namespace stanley
} // namespace movement