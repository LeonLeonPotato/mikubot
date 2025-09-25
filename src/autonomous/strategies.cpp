#include "autonomous/strategies.h"
#include "ansicodes.h"
#include "pros/rtos.hpp"
#include <iostream>

using namespace strategies;
using namespace std;

static void none(void) {
    cout << PREFIX << "No strategy chosen" << std::endl;
    pros::delay(500);
}

const map<Strategy, std::string> strategies::names = {
    {Strategy::Test, "Test"},
    {Strategy::MPCStrat, "MPC Strat"},
    {Strategy::Skills, "Skills"},
    {Strategy::None, "None"}
};

const map<std::string, Strategy> strategies::values = {
    {"Test", Strategy::Test},
    {"MPC Strat", Strategy::MPCStrat},
    {"Skills", Strategy::Skills},
    {"None", Strategy::None}
};

const map<Strategy, void (*)()> strategies::functions = {
    {Strategy::Test, test_strategy::run},
    {Strategy::MPCStrat, mpcstrat::run},
    {Strategy::Skills, skills::run},
    {Strategy::None, none}
};

Strategy strategies::chosen_strategy = Strategy::Test;
