#pragma once

#include "opcontrol/impl/driving.h"
#include "opcontrol/impl/intake.h"
#include "opcontrol/impl/conveyor.h"
#include "opcontrol/test/lidartest.h"

#include <vector>
#include <functional>

namespace controls {
const extern std::vector<std::function<void(void)>> ticks;
const extern std::vector<std::function<void(void)>> runs;
const extern std::vector<std::function<void(void)>> start_tasks;
const extern std::vector<std::function<void(void)>> stop_tasks;
}