#pragma once

#include "autonomous/pathing/quintic_spline.h"
#include "autonomous/pathing/polygon_path.h"

namespace pathing {
void init(void) {
    QuinticSpline::init();
}
} // namespace pathing