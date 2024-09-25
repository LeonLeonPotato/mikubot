#include "gui/visiontest.h"
#include "gui/utils.h"
#include "gui/gif-pros/gifclass.hpp"
#include "autonomous/strategies.h"
#include "version.h"
#include "robot.h"
#include <map>

#include "api.h"
#include "liblvgl/lvgl.h"
#include "gui/fonts/roboto_regular_16.c"
#include "gui/fonts/roboto_regular_bold_16.c"
#include "gui/fonts/roboto_regular_30.c"

namespace visiontest {
pros::task_t task;

std::map<int, int> sig_color_map;

struct bbox {
    lv_obj_t* box;
    lv_style_t* style;
    lv_obj_t* sig;
    lv_style_t* sig_style;
};

std::vector<bbox> boxes;

int x, y;
lv_obj_t* view_box;
lv_style_t* view_box_style;

lv_obj_t* obj_cnt;

void destroy_box(bbox& b) {
    lv_obj_del(b.box);
    lv_style_reset(b.style);
    lv_obj_del(b.sig);
    lv_style_reset(b.sig_style);
}

bbox create_box(void) {
    bbox b;
    b.box = lv_obj_create(view_box);
    b.style = new lv_style_t();
    lv_style_init(b.style);
    lv_style_set_bg_opa(b.style, LV_OPA_0);
    lv_style_set_border_width(b.style, 1);

    b.sig = lv_obj_create(view_box);
    b.sig_style = new lv_style_t();
    lv_style_init(b.sig_style);
    lv_style_set_bg_opa(b.sig_style, LV_OPA_50);
}

void update(void) {
    int cnt = robot::vision.get_object_count();
    lv_label_set_text_fmt(obj_cnt, "Objects: %d", cnt);

    while (boxes.size() > cnt) {
        destroy_box(boxes.back());
        boxes.pop_back();
    }
    for (int i = 0; i < cnt; i++) {
        if (i >= boxes.size()) boxes.push_back(create_box());
        bbox& b = boxes[i];
        pros::vision_object_s_t obj = robot::vision.get_by_sig(0, i);
        lv_obj_set_pos(boxes[i].box, obj.left_coord, obj.top_coord);
        lv_obj_set_size(boxes[i].box, obj.width, obj.height);
        lv_style_set_border_color(b.style, lv_color_hex(sig_color_map[obj.signature]));

        lv_obj_set_pos(boxes[i].sig, obj.left_coord, obj.top_coord - 20);
        std::string text = "Sig " + std::to_string(obj.signature);
        lv_label_set_text(boxes[i].sig, text.c_str());
    }
}

void init(void) {
    view_box = lv_obj_create(lv_scr_act());
    x = 100; y = 10;
    lv_obj_remove_style_all(view_box);
    lv_obj_set_pos(view_box, x, y);
    lv_obj_set_size(view_box, LV_HOR_RES_MAX - 10 - x, LV_VER_RES_MAX - 10 - y);

    view_box_style = new lv_style_t();
    lv_style_init(view_box_style);
    lv_style_set_bg_opa(view_box_style, LV_OPA_0);
    lv_style_set_border_color(view_box_style, lv_color_hex(0xFFFFFF));
    lv_style_set_border_width(view_box_style, 1);
    lv_obj_add_style(view_box, view_box_style, 0);

    obj_cnt = lv_label_create(lv_scr_act());
    lv_label_set_text(obj_cnt, "Objects: 0");
    lv_obj_set_pos(view_box, 10, 10);

    sig_color_map[0] = 0xFF0000;
    sig_color_map[1] = 0x00FF00;
    sig_color_map[2] = 0x0000FF;
    sig_color_map[3] = 0xFFFF00;
    sig_color_map[4] = 0xFF00FF;
    sig_color_map[5] = 0x00FFFF;
    sig_color_map[6] = 0xFFFFFF;
    
    task = pros::c::task_create([](void*) {
        while (true) {
            update();
            pros::c::task_delay(50);
        }
    }, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Test");
}

void destroy(void) {
    pros::c::task_suspend(task);
    lv_obj_del(view_box);
    lv_style_reset(view_box_style);
    lv_obj_del(obj_cnt);
    for (bbox& b : boxes) destroy_box(b);
    boxes.clear();
    sig_color_map.clear();
}
}
