#include "gui/simtest.h"
#include "liblvgl/lvgl.h"

static lv_obj_t* my_rect;

void simtest::init() {
    my_rect = lv_obj_create(lv_scr_act());
    lv_obj_set_size(my_rect , 50, 50);
    lv_obj_set_pos(my_rect , 25, 25);
    lv_obj_set_style_bg_color(my_rect, lv_color_hex(0xFFFFFF), 0);
}

void simtest::destroy() {
    lv_obj_del(my_rect);
}