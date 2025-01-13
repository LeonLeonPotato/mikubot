#include "autonomous/strategies.h"
#include <iostream>

using namespace strategies;

static void none(
    
) {
    std::cout << "No strategy chosen" << std::endl;
}

const std::map<Strategy, std::string> strategies::names = {
    {Strategy::Test, "Test"},
    {Strategy::PeakStrat, "Peak Strat"},
    {Strategy::None, "None"}
};

const std::map<std::string, Strategy> strategies::values = {
    {"Test", Strategy::Test},
    {"Peak Strat", Strategy::PeakStrat},
    {"None", Strategy::None}
};

const std::map<Strategy, void (*)()> strategies::functions = {
    {Strategy::Test, test_strategy::run},
    {Strategy::PeakStrat, peak_strat::run},
    {Strategy::None, none}
};

Strategy strategies::chosen_strategy = Strategy::Test;