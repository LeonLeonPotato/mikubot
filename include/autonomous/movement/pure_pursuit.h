#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "autonomous/solvers.h"
#include "autonomous/pathing/base_path.h"
#include "autonomous/controllers.h"

namespace movement {
namespace pure_pursuit {

float follow_path_tick(pathing::BasePath& path, controllers::PID& pid, float t, float radius,
                        solvers::func_t func, solvers::func_t deriv, int iterations = 5);
void follow_path(pathing::BasePath& path, float radius, int iterations = 5);

} // namespace pure_pursuit
} // namespace movement