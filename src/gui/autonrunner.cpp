#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/strategies.h"
#include "gui/fonts/roboto_regular_24.c"
#include "essential.h"

#include "api.h"
#include "liblvgl/lvgl.h"

namespace autonrunner {
renderer::Text* text;

void init(void) {
    using namespace strategies::config;

    text = new renderer::Text("Running auton", roboto_regular_24, 0, 0, 0xFFFFFF);
    std::string st = "Running " + strategies::names.at(chosen_strategy) + " strategy\n" + \
        "on team " + get_team_name() + "\n" + \
        "on side " + get_side_name() + "\n\n" + \
        "Multiplier: " + std::to_string(get_multiplier());
    text->rename(st);
}

void destroy(void) {
    delete text;
}
}