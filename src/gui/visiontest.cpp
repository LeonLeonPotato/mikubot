#include "gui/visiontest.h"
#include "gui/utils.h"
#include "essential.h"
#include <map>

#include "api.h"
#include "liblvgl/lvgl.h"

namespace visiontest {
pros::task_t task;

std::map<int, int> sig_color_map;

struct bbox {
    lv_obj_t* box;
    lv_style_t* style;
    lv_obj_t* sig;
    lv_style_t* sig_style;
};

std::vector<bbox*> boxes;

int x, y;
lv_obj_t* view_box;
lv_style_t* view_box_style;

lv_obj_t* obj_cnt;

void destroy_box(bbox* b) {
    lv_obj_del(b->box);
    lv_style_reset(b->style);
    lv_obj_del(b->sig);
    lv_style_reset(b->sig_style);
}

bbox* create_box(void) {
    bbox* b = new bbox();
    b->box = lv_obj_create(view_box);
    lv_obj_remove_style_all(b->box);

    b->style = new lv_style_t();
    lv_style_init(b->style);
    lv_style_set_bg_opa(b->style, LV_OPA_0);
    lv_style_set_border_width(b->style, 1);
    lv_obj_add_style(b->box, b->style, 0);

    b->sig = lv_label_create(view_box);
    b->sig_style = new lv_style_t();
    lv_style_init(b->sig_style);
    lv_style_set_bg_opa(b->sig_style, LV_OPA_0);
    lv_style_set_text_color(b->sig_style, lv_color_hex(0xFFFFFF));
    lv_obj_add_style(b->sig, b->sig_style, 0);
    return b;
}

void update(void* args) {
    for (int i = 0; i < 10; i++) {
        boxes.push_back(create_box());
    }

    std::cout << "Starting vision test" << std::endl;

    while (true) {
        // int cnt = robot::vision.get_object_count();
        int cnt = 0;
        lv_label_set_text_fmt(obj_cnt, "Objects: %d", cnt);

        pros::vision_object_s_t object_arr[10];
        // int n = robot::vision.read_by_size(0, 10, object_arr);
        int n = 0;

        for (int i = 0; i < std::min(10, cnt); i++) {
            bbox* b = boxes[i];
            pros::vision_object_s_t& obj = object_arr[i];

            lv_obj_set_pos(b->box, obj.left_coord, obj.top_coord);
            lv_obj_set_size(b->box, obj.width, obj.height);
            lv_style_set_border_color(b->style, lv_color_hex(sig_color_map[obj.signature]));

            lv_obj_set_pos(b->sig, obj.left_coord, obj.top_coord - 20);
            std::string text = "Sig " + std::to_string(obj.signature);
            lv_label_set_text(b->sig, text.c_str());

            printf("Sig %d at [%d, %d, %d, %d]\n", obj.signature, obj.left_coord, obj.top_coord, obj.width, obj.height);
        }
        // for (int i = cnt; i < boxes.size(); i++) {
        //     bbox* b = boxes[i];
        //     lv_obj_set_pos(b->box, 0, 0);
        //     lv_obj_set_size(b->box, 1, 1);
        //     lv_label_set_text(b->sig, "");
        // }

        pros::c::task_delay(50);
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
    lv_obj_set_pos(obj_cnt, 10, 10);

    sig_color_map[robot::signatures::blue_ring_id] = 0x0000FF;
    sig_color_map[robot::signatures::red_ring_id] = 0xFF0000;
    sig_color_map[robot::signatures::goal_id] = 0x00FF00;
    
    task = pros::c::task_create(update, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Test");
}

void destroy(void) {
    pros::c::task_suspend(task);
    lv_obj_del(view_box);
    lv_style_reset(view_box_style);
    lv_obj_del(obj_cnt);
    for (bbox*& b : boxes) {
        destroy_box(b);
        delete b;
    }
    boxes.clear();
    sig_color_map.clear();
}
}