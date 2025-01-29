#pragma once

#include "pose.h"
#include "simpletils.h"

namespace movement::simple {

DECLARE_ALL(turn_towards, controllers::PID&, const float angle)
DECLARE_ALL(turn_towards, controllers::PID&, const Eigen::Vector2f& point)
DECLARE_ALL(turn_towards, controllers::PID&, const Pose& point)

}