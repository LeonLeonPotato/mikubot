#include "autonomous/strategies.h"

using namespace strategies;

const std::map<Strategy, std::string> strategies::names = {
    {Strategy::TEST_1, "Test"}
};

const std::map<std::string, Strategy> strategies::values = {
    {"Test", Strategy::TEST_1}
};

const std::map<Strategy, void (*)()> strategies::functions = {
    {Strategy::TEST_1, test_strategy::run}
};

Strategy strategies::chosen_strategy = Strategy::TEST_1;