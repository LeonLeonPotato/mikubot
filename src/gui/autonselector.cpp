#include "gui/autonselector.h"
#include "gui/utils.h"
#include "gui/gif-pros/gifclass.hpp"
#include "autonomous/strategies.h"
#include "version.h"
#include "robot.h"

#include "api.h"
#include "liblvgl/lvgl.h"
#include "gui/fonts/roboto_regular_16.c"
#include "gui/fonts/roboto_regular_bold_16.c"
#include "gui/fonts/roboto_regular_30.c"

#include <vector>

namespace autonselector {
struct Auton_strategy_button {
    strategies::Strategy strategy;
    lv_obj_t* button;
    lv_style_t* button_style;
    lv_obj_t* text;
    lv_style_t* text_style;
};

bool finished_selection = false;

lv_obj_t* logo;

lv_obj_t* team_selection_text;
lv_style_t* team_selection_text_style;
lv_obj_t* team_button;
lv_style_t* team_button_style;
lv_obj_t* team_button_text;
lv_style_t* team_button_text_style;

lv_obj_t* auton_selection_text;
lv_style_t* auton_selection_text_style;
lv_obj_t* selected_auton_box;
lv_style_t* selected_auton_box_style;
Auton_strategy_button* selected_auton_strategy;
std::vector<Auton_strategy_button*> auton_buttons;

lv_obj_t* confirm_button;
lv_style_t* confirm_button_style;
lv_obj_t* confirm_button_text;

Gif* miku_gif;

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

void team_switch_callback(lv_event_t* e) {
    if (*lv_label_get_text(team_button_text) == 'R') { // hack
        lv_label_set_text(team_button_text, "Blue");
        lv_style_set_bg_color(team_button_style, lv_color_hex(0x0000FF));
    } else {
        lv_label_set_text(team_button_text, "Red");
        lv_style_set_bg_color(team_button_style, lv_color_hex(0xFF0000));
    }
}

inline void team_selector(void) {
    team_selection_text = lv_label_create(lv_scr_act());;
    lv_label_set_text(team_selection_text, "Team");
    lv_obj_set_pos(team_selection_text, 10, 70);

    team_selection_text_style = new lv_style_t();
    lv_style_init(team_selection_text_style);
    lv_style_set_text_font(team_selection_text_style, &roboto_regular_30);
    lv_style_set_text_color(team_selection_text_style, lv_color_hex(0xFFFFFF));
    lv_obj_add_style(team_selection_text, team_selection_text_style, 0);

    ////////// Team Button //////////

    team_button = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(team_button, 100, 60);
    lv_obj_set_size(team_button, 130, 50);

    team_button_style = new lv_style_t();
    lv_style_init(team_button_style);
    lv_style_set_bg_color(team_button_style, lv_color_hex(0xFF0000));
    lv_obj_add_style(team_button, team_button_style, 0);
    
    team_button_text = lv_label_create(team_button);
    lv_label_set_text(team_button_text, "Red");
    lv_obj_center(team_button_text);

    team_button_text_style = new lv_style_t();
    lv_style_init(team_button_text_style);
    lv_style_set_text_color(team_button_text_style, lv_color_hex(0xFFFFFF));
    lv_style_set_text_font(team_button_text_style, &roboto_regular_16);
    lv_obj_add_style(team_button_text, team_button_text_style, 0);

    lv_obj_add_event_cb(team_button, team_switch_callback, LV_EVENT_CLICKED, NULL);
}

void auton_selection_callback(lv_event_t* e) {
    Auton_strategy_button* clicked_button = static_cast<Auton_strategy_button*>(lv_event_get_user_data(e));

    lv_obj_set_pos(
        selected_auton_box, 
        lv_obj_get_x(clicked_button->button)-5, 
        lv_obj_get_y(clicked_button->button)-5
    );

    lv_style_set_text_font(clicked_button->text_style, &roboto_regular_bold_16);
    lv_style_set_text_font(selected_auton_strategy->text_style, &roboto_regular_16);

    selected_auton_strategy = clicked_button;
}

inline void auton_strategy(void) {
    auton_selection_text = lv_label_create(lv_scr_act());;
    lv_label_set_text(auton_selection_text, "Auton");
    lv_obj_set_pos(auton_selection_text, 10, 130);

    auton_selection_text_style = new lv_style_t();
    lv_style_init(auton_selection_text_style);
    lv_style_set_text_font(auton_selection_text_style, &roboto_regular_30);
    lv_style_set_text_color(auton_selection_text_style, lv_color_hex(0xFFFFFF));
    lv_obj_add_style(auton_selection_text, auton_selection_text_style, 0);

    int x = 115;
    int y = 120;
    for (auto& i : strategies::names) {
        Auton_strategy_button* auton_button = new Auton_strategy_button();
        auton_button->strategy = i.first;

        auton_button->button = lv_btn_create(lv_scr_act());
        lv_obj_set_pos(auton_button->button , x, y);
        lv_obj_set_size(auton_button->button , 130, 50);

        auton_button->button_style = new lv_style_t();
        lv_style_init(auton_button->button_style);
        lv_style_set_bg_color(auton_button->button_style, lv_color_hex(0x34aeeb));
        lv_obj_add_style(auton_button->button, auton_button->button_style, 0);

        auton_button->text = lv_label_create(auton_button->button);
        lv_label_set_text(auton_button->text, i.second.c_str());
        lv_obj_center(auton_button->text);

        auton_button->text_style = new lv_style_t();
        lv_style_init(auton_button->text_style);
        lv_style_set_text_color(auton_button->text_style, lv_color_hex(0xFFFFFF));
        lv_style_set_text_font(auton_button->text_style, &roboto_regular_16);
        lv_obj_add_style(auton_button->text, auton_button->text_style, 0);

        auton_buttons.push_back(auton_button);
        lv_obj_add_event_cb(auton_button->button, auton_selection_callback, LV_EVENT_CLICKED, auton_button);

        if (i.first == strategies::default_strategy) {
            lv_style_set_text_font(auton_button->text_style, &roboto_regular_bold_16);

            selected_auton_box = lv_obj_create(lv_scr_act());
            lv_obj_remove_style_all(selected_auton_box);
            lv_obj_set_pos(selected_auton_box, x-5, y-5);
            lv_obj_set_size(selected_auton_box, 140, 60);

            selected_auton_box_style = new lv_style_t();
            lv_style_init(selected_auton_box_style);
            lv_style_set_border_color(selected_auton_box_style, lv_color_hex(0xFFFFFF));
            lv_style_set_border_width(selected_auton_box_style, 2);
            lv_style_set_bg_opa(selected_auton_box_style, LV_OPA_0);
            lv_obj_add_style(selected_auton_box, selected_auton_box_style, 0);

            selected_auton_strategy = auton_button;
        }

        x += 140;
        if (x + 130 > LV_HOR_RES_MAX - 10) {
            x = 115;
            y += 60;
        }
    }
}

void confirm_selection_callback(lv_event_t* e) {
    finished_selection = true;

    robot::team = *lv_label_get_text(team_button_text);
    robot::auton_strategy = selected_auton_strategy->strategy;
}

inline void confirm_selection(void) {
    confirm_button = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(confirm_button, 20, 175);
    lv_obj_set_size(confirm_button, 70, 50);

    confirm_button_style = new lv_style_t();
    lv_style_init(confirm_button_style);
    lv_style_set_bg_color(confirm_button_style, lv_color_hex(0x34aeeb));
    lv_obj_add_style(confirm_button, confirm_button_style, 0);

    confirm_button_text = lv_label_create(confirm_button);
    lv_label_set_text(confirm_button_text, "Confirm");
    lv_obj_center(confirm_button_text);

    lv_obj_add_event_cb(confirm_button, confirm_selection_callback, LV_EVENT_CLICKED, NULL);
}

void init_gif(void) {
    miku_gif = new Gif("/usd/kaito-miku-90-nobg.gif", lv_scr_act(), 380, 10);
}

void init(void) {
    create_logo();
    team_selector();
    auton_strategy();
    confirm_selection();
    init_gif();
}

void destroy(void) {
    lv_obj_del(logo);

    lv_obj_del(team_selection_text);
    lv_obj_del(team_button);
    lv_style_reset(team_button_style);
    lv_style_reset(team_button_text_style);
    lv_style_reset(team_selection_text_style);


    lv_obj_del(auton_selection_text);
    lv_obj_del(selected_auton_box);
    lv_style_reset(selected_auton_box_style);
    lv_style_reset(auton_selection_text_style);

    for (auto& i : auton_buttons) {
        lv_obj_del(i->button);
        lv_style_reset(i->button_style);
        lv_style_reset(i->text_style);
        delete i;
    }

    lv_obj_del(confirm_button);
    lv_style_reset(confirm_button_style);

    miku_gif->clean();
    delete miku_gif;
}
}