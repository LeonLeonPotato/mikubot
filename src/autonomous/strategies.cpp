#include "autonomous/strategies.h"
#include "ansicodes.h"
#include "pros/rtos.hpp"
#include <iostream>

using namespace strategies;

static void none(void) {
    std::cout << PREFIX << "No strategy chosen" << std::endl;
    pros::delay(500);
}

const std::map<Strategy, std::string> strategies::names = {
    {Strategy::Test, "Test"},
    {Strategy::MPCStrat, "MPC Strat"},
    {Strategy::None, "None"}
};

const std::map<std::string, Strategy> strategies::values = {
    {"Test", Strategy::Test},
    {"MPC Strat", Strategy::MPCStrat},
    {"None", Strategy::None}
};

const std::map<Strategy, void (*)()> strategies::functions = {
    {Strategy::Test, test_strategy::run},
    {Strategy::MPCStrat, mpcstrat::run},
    {Strategy::None, none}
};

Strategy strategies::chosen_strategy = Strategy::Test;