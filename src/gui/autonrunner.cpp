#include "gui/autonrunner.h"
#include "gui/utils.h"
#include "autonomous/strategies.h"
#include "gui/fonts/roboto_regular_24.c"
#include "essential.h"

#include "api.h"  // IWYU pragma: keep
#include "liblvgl/lvgl.h" // IWYU pragma: keep

bool initialized = false;
renderer::Text* text;

void autonrunner::init(void) {
    if (initialized) return;
    initialized = true;

    text = new renderer::Text("Running auton", roboto_regular_24, 0, 0, 0xFFFFFF);

    char st[256]; memset(st, 0, sizeof(st));
    sprintf(st, "Running %s\nTeam: %s\nSide: %s", 
        strategies::names.at(strategies::chosen_strategy).c_str(),
        robot::match::get_team_name().c_str(),
        robot::match::get_side_name().c_str()
    );
    text->rename(st);
}

void autonrunner::destroy(void) {
    if (!initialized) return;
    initialized = false;

    delete text;
}