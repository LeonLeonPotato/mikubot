#include "autonomous/strategies.h"
#include "autonomous/strategy/playback.h"
#include "autonomous/strategy/test.h"

#include <unordered_map>
#include <string>

namespace strategies {
const std::unordered_map<Strategy, std::string> names = {
    {Strategy::Playback, "Playback"},
    {Strategy::Skills, "Skills"},
    {Strategy::TEST_1, "Test"}
};

const std::unordered_map<std::string, Strategy> values = {
    {"Playback", Strategy::Playback},
    {"Skills", Strategy::Skills},
    {"TEST_1", Strategy::TEST_1}
};

const std::unordered_map<Strategy, void (*)()> functions = {
    {Strategy::Playback, nullptr},
    {Strategy::Skills, nullptr},
    {Strategy::TEST_1, test_strategy::run}
};

const Strategy default_strategy = Strategy::TEST_1;

namespace config {
strategies::Strategy chosen_strategy = strategies::default_strategy;
char team = 'R';
int side = 1;
} // namespace config

} // namespace strategies