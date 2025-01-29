#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple/simpletils.h"
#include "pose.h"

namespace movement::simple {

DECLARE_ALL(boomerang, PIDGroup,
    const Pose& pose,
    const float lead)

}