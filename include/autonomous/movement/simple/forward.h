#pragma once

#include "autonomous/controllers/pid.h"
#include "autonomous/movement/simple/simpletils.h"

namespace movement::simple {

DECLARE_TICK(forward, controllers::PID&, const Eigen::Vector2f& line, const Eigen::Vector2f& normal);
DECLARE_CANCELLABLE(forward, controllers::PID&, const float cm);
DECLARE_STANDARD(forward, controllers::PID&, const float cm);
DECLARE_ASYNC(forward, controllers::PID&, const float cm);

}