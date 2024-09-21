#include "gui/autonselector.h"
#include "gui/utils.h"
#include "version.h"

#include "api.h"
#include "liblvgl/lvgl.h"
#include "gui/fonts/roboto_regular_16.c"
#include "gui/fonts/roboto_regular_24.c"
#include "gui/fonts/roboto_regular_30.c"

namespace autonselector {
lv_obj_t* logo; 

lv_obj_t* team_selection_text;
lv_obj_t* red_team_button;
lv_obj_t* blue_team_button;

inline void create_logo(void) {
    logo = lv_spangroup_create(lv_scr_act());
    lv_obj_set_pos(logo, 10, 10);

    lv_span_t* big_m = lv_spangroup_new_span(logo);;
    const char m = 'M';
    lv_span_set_text(big_m, &m);

    lv_style_init(&big_m->style);
    lv_style_set_text_color(&big_m->style, lv_color_hex(0x34aeeb));
    lv_style_set_text_font(&big_m->style, &roboto_regular_30);

    lv_span_t* logo_rest = lv_spangroup_new_span(logo);
    std::string remaining_text = "ikuBot version " + std::string(VERSION_STRING);
    lv_span_set_text(logo_rest, remaining_text.c_str());

    lv_style_init(&logo_rest->style);
    lv_style_set_text_color(&logo_rest->style, lv_color_hex(0xFFFFFF));
    lv_style_set_text_font(&logo_rest->style, &roboto_regular_16);

    lv_spangroup_refr_mode(logo);
}

void button_event_handler(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* label = static_cast<lv_obj_t*>(lv_event_get_user_data(e));
    lv_obj_t* btn = lv_event_get_target(e);

    if (code != LV_EVENT_CLICKED) {
        return;
    }

    if (btn == red_team_button) {
        std::cout << "Red Team" << std::endl;
    } else if (btn == blue_team_button) {
        std::cout << "Blue Team" << std::endl;
    }
}

inline void team_selector(void) {
    team_selection_text = lv_label_create(lv_scr_act());;
    lv_label_set_text(team_selection_text, "Team");
    lv_obj_set_pos(team_selection_text, 10, 70);

    static lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_font(&label_style, &roboto_regular_30);
    lv_style_set_text_color(&label_style, lv_color_hex(0xFFFFFF));
    lv_obj_add_style(team_selection_text, &label_style, LV_PART_MAIN | LV_STATE_DEFAULT);

    ////////// Red Team Button //////////

    red_team_button = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(red_team_button, 100, 60);
    lv_obj_set_size(red_team_button, 130, 50);

    static lv_style_t red_button_style;
    lv_style_init(&red_button_style);
    lv_style_set_bg_color(&red_button_style, lv_color_hex(0xFF0000));
    lv_obj_add_style(red_team_button, &red_button_style, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    lv_obj_t* red_label = lv_label_create(red_team_button);
    lv_label_set_text(red_label, "Red");
    lv_obj_center(red_label);

    static lv_style_t red_label_style;
    lv_style_init(&red_label_style);
    lv_style_set_text_color(&red_label_style, lv_color_hex(0xFFFFFF));
    lv_style_set_text_font(&red_label_style, &roboto_regular_16);
    lv_obj_add_style(red_label, &red_label_style, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(red_team_button, button_event_handler, LV_EVENT_ALL, red_label);

    ////////// Blue Team Button //////////
}

void init(void) {
    create_logo();
    team_selector();
}

void destroy(void) {
    lv_obj_del(logo);
    lv_obj_del(team_selection_text);
    lv_obj_del(red_team_button);
}
}