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

namespace config {
extern strategies::Strategy chosen_strategy;
extern char team;
extern int side;

inline int get_multiplier(void) {
    return team == 'R' ? side : -side;
}

inline std::string get_team_name(void) {
    return team == 'R' ? "Red" : "Blue";
}

inline std::string get_side_name(void) {
    return side == 1 ? "left" : "right";
}
}
} // namespace strategies