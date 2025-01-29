#pragma once

#include "autonomous/movement/base_movement.h"
#include "pose.h"
#include "simpletils.h"

namespace movement::simple {

DECLARE_ALL(swing_to, PIDGroup, const Eigen::Vector2f& point)
DECLARE_ALL(swing_to, PIDGroup, const Pose& pose)

}