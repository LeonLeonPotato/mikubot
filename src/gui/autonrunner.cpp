#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/autonconfig.h"
#include "gui/fonts/roboto_regular_24.c"
#include "essential.h"

#include "api.h"
#include "liblvgl/lvgl.h"

namespace autonrunner {
renderer::Text* text;

void init(void) {
    text = new renderer::Text("Running auton", roboto_regular_24, 0, 0, 0xFFFFFF);
    std::string st = "Running " + auton_strategies::names.at(auton_config::chosen_strategy) + " strategy\n" + \
        "on team " + auton_config::get_team_name() + "\n" + \
        "on side " + auton_config::get_side_name() + "\n\n" + \
        "Multiplier: " + std::to_string(auton_config::get_multiplier());
    text->rename(st);
}

void destroy(void) {
    delete text;
}
}