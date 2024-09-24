#pragma once

#include "autonomous/strategy/playback.h"
#include "autonomous/strategy/test.h"

#include <unordered_map>
#include <string>

namespace strategies {
    enum class Strategy {
        Playback,
        Skills,
        TEST_1
    };

    extern const std::unordered_map<Strategy, std::string> names;
    extern const std::unordered_map<std::string, Strategy> values;
    extern const std::unordered_map<Strategy, void (*)()> functions;
    extern const Strategy default_strategy;
}