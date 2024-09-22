#include "autonomous/strategies.h"

#include <unordered_map>
#include <string>

namespace strategies {
const std::unordered_map<Strategy, std::string> names = {
    {Strategy::Playback, "Playback"},
    {Strategy::Skills, "Skills"},
    {Strategy::TEST_1, "TEST_1"}
};

const std::unordered_map<std::string, Strategy> values = {
    {"Playback", Strategy::Playback},
    {"Skills", Strategy::Skills},
    {"TEST_1", Strategy::TEST_1}
};

const Strategy default_strategy = Strategy::Skills;
}