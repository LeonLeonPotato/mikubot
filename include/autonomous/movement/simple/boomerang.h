#pragma once

#include "autonomous/movement/base_movement.h"
#include "autonomous/movement/simple/simpletils.h"

namespace movement::simple {

DECLARE_ALL(boomerang, PIDGroup,
    const Eigen::Vector2f& point, 
    const float angle,
    const float lead)

}