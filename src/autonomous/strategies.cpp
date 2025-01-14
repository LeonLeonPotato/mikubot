#include "autonomous/strategies.h"
#include "ansicodes.h"
#include "pros/rtos.hpp"
#include "strategy/bcis.h"
#include <iostream>

using namespace strategies;

static void none(void) {
    std::cout << PREFIX << "No strategy chosen" << std::endl;
    pros::delay(500);
}

const std::map<Strategy, std::string> strategies::names = {
    {Strategy::Test, "Test"},
    {Strategy::PeakStrat, "Peak Strat"},
    {Strategy::BCIS, "BCIS"},
    {Strategy::None, "None"}
};

const std::map<std::string, Strategy> strategies::values = {
    {"Test", Strategy::Test},
    {"Peak Strat", Strategy::PeakStrat},
    {"BCIS", Strategy::BCIS},
    {"None", Strategy::None}
};

const std::map<Strategy, void (*)()> strategies::functions = {
    {Strategy::Test, test_strategy::run},
    {Strategy::PeakStrat, peak_strat::run},
    {Strategy::BCIS, bcis::run},
    {Strategy::None, none}
};

Strategy strategies::chosen_strategy = Strategy::Test;