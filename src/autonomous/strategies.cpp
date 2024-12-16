#include "autonomous/strategies.h"

using namespace strategies;

const std::map<Strategy, std::string> strategies::names = {
    {Strategy::Test, "Test"},
    {Strategy::PeakStrat, "Peak Strat"}
};

const std::map<std::string, Strategy> strategies::values = {
    {"Test", Strategy::Test},
    {"Peak Strat", Strategy::PeakStrat}
};

const std::map<Strategy, void (*)()> strategies::functions = {
    {Strategy::Test, test_strategy::run},
    {Strategy::PeakStrat, peak_strat::run}
};

Strategy strategies::chosen_strategy = Strategy::Test;