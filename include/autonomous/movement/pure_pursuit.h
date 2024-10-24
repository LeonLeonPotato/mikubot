#pragma once

#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"

namespace movement {
namespace pure_pursuit {

float follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, float radius,
                        solvers::func_t func, solvers::func_t deriv, 
                        solvers::func_vec_t vec_func, solvers::func_vec_t vec_deriv, 
                        float end_heading = 0, float end_magnitude = 0,
                        int iterations = 5);

float follow_path(pathing::BasePath& path,
                float radius,
                controllers::PID* pid = nullptr,
                float end_heading = 0, float end_magnitude = 0,
                int iterations = 5, long long timeout = 5000);

} // namespace pure_pursuit
} // namespace movement