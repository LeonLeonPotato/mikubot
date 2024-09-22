#include "gui/autonrunner.h"
#include "autonomous/strategies.h"
#include "robot.h"

#include "api.h"
#include "liblvgl/lvgl.h"

namespace autonrunner {

lv_obj_t* text;

void init(void) {
    text = lv_label_create(lv_scr_act());
    const std::string team = robot::team == 'R' ? "Red" : "Blue";
    const std::string st = "Autonomous running strategy " + strategies::names.at(robot::auton_strategy) + " for team " + team;
    lv_label_set_text(text, st.c_str());
    lv_obj_align(text, LV_ALIGN_CENTER, 0, 0);
}

void destroy(void) {
    lv_obj_del(text);
}
}