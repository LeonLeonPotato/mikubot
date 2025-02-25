#pragma once

#include "autonomous/strategy/test.h"
#include "autonomous/strategy/mpcstrat.h"
#include "autonomous/strategy/skills.h"

#include <map>
#include <string>

namespace strategies {
enum class Strategy {
    Test,
    MPCStrat,
    Skills,
    None
};

extern const std::map<Strategy, std::string> names;
extern const std::map<std::string, Strategy> values;
extern const std::map<Strategy, void (*)()> functions;

extern strategies::Strategy chosen_strategy;
} // namespace strategies