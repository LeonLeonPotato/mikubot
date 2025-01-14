#pragma once

#include "autonomous/strategy/peak_strat.h"
#include "autonomous/strategy/test.h"

#include <map>
#include <string>

namespace strategies {
enum class Strategy {
    PeakStrat,
    Test,
    BCIS,
    None
};

extern const std::map<Strategy, std::string> names;
extern const std::map<std::string, Strategy> values;
extern const std::map<Strategy, void (*)()> functions;

extern strategies::Strategy chosen_strategy;
} // namespace strategies