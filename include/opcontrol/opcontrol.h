#pragma once

#include "opcontrol/impl/driving.h" // IWYU pragma: export
#include "opcontrol/impl/intake.h" // IWYU pragma: export
#include "opcontrol/impl/conveyor.h" // IWYU pragma: export
#include "opcontrol/test/lidartest.h" // IWYU pragma: export
#include "opcontrol/impl/clamp.h" // IWYU pragma: export
#include "opcontrol/impl/ejector.h" // IWYU pragma: export
#include "opcontrol/impl/wallmech.h" // IWYU pragma: export
#include "opcontrol/impl/doinker.h" // IWYU pragma: export
#include "opcontrol/test/odom_center.h" // IWYU pragma: export

#include <vector>
#include <functional>

namespace controls {
const extern std::vector<std::function<void(void)>> ticks;
const extern std::vector<std::function<void(void)>> runs;
const extern std::vector<std::function<void(void)>> start_tasks;
const extern std::vector<std::function<void(void)>> stop_tasks;
}