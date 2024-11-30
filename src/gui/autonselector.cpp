#include "gui/autonselector.h"
#include "gui/utils.h"
#include "autonomous/strategies.h"
#include "version.h"
#include "essential.h"

#include "api.h"
#include "liblvgl/lvgl.h"
#include "gui/fonts/roboto_regular_16.c"
#include "gui/fonts/roboto_regular_bold_16.c"
#include "gui/fonts/roboto_regular_30.c"

#include <vector>

namespace autonselector {
bool initialized = false;
bool finished_selection = false;

lv_obj_t* logo;

renderer::Text* team_selection_text;
renderer::NamedButton* team_button;
renderer::NamedButton* side_button;

renderer::Text* auton_selection_text;
int current_selected_idx;
std::vector<std::pair<strategies::Strategy, renderer::NamedButton*>> auton_buttons;
lv_obj_t* selected_auton_box;
lv_style_t* selected_auton_box_style;

renderer::NamedButton* confirm_button;

lv_obj_t* miku_gif;

void create_logo(void) {
    logo = lv_spangroup_create(lv_scr_act());
    lv_obj_set_pos(logo, 10, 10);

    lv_span_t* big_m = lv_spangroup_new_span(logo);;
    lv_span_set_text(big_m, "M");

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
    if (robot::match::team == 'R') {
        robot::match::team = 'B';
        team_button->bg_color(0x0000FF);
    } else {
        robot::match::team = 'R';
        team_button->bg_color(0xFF0000);
    }
    team_button->rename(robot::match::get_team_name());
}

void side_switch_callback(lv_event_t* e) {
    robot::match::side = -robot::match::side;
    side_button->rename(robot::match::get_side_name());
}

void team_selector(void) {
    team_selection_text = new renderer::Text(
        "Team",
        roboto_regular_30, 
        10, 70
    );
    team_button = new renderer::NamedButton(
        robot::match::get_team_name(), 
        roboto_regular_bold_16, 
        110, 60, 120, 50, 
        0xFF0000
    );
    side_button = new renderer::NamedButton(
        robot::match::get_side_name(), 
        roboto_regular_bold_16, 
        240, 60, 120, 50,
        0x838482
    );

    lv_obj_add_event_cb(team_button->button, team_switch_callback, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(side_button->button, side_switch_callback, LV_EVENT_CLICKED, NULL);
}

void auton_selection_callback(lv_event_t* e) {
    int i = reinterpret_cast<int>(lv_event_get_user_data(e));
    const auto& clicked_button = auton_buttons[i].second;
    const auto& clicked_strategy = auton_buttons[i].first;

    lv_obj_set_pos(
        selected_auton_box, 
        lv_obj_get_x(clicked_button->button)-5, 
        lv_obj_get_y(clicked_button->button)-5
    );

    clicked_button->set_font(roboto_regular_bold_16);
    auton_buttons[current_selected_idx].second->set_font(roboto_regular_16);

    current_selected_idx = i;
}

void auton_strategy(void) {
    auton_selection_text = new renderer::Text(
        "Auton",
        roboto_regular_30, 
        10, 130
    );

    int x = 115;
    int y = 120;
    int i = 0;
    for (const auto& strat : strategies::names) {
        auto* btn = new renderer::NamedButton(
            strat.second, 
            roboto_regular_16, 
            x, y, 130, 50
        );
        lv_obj_add_event_cb(btn->button, auton_selection_callback, LV_EVENT_CLICKED, reinterpret_cast<void*>(i));

        auton_buttons.emplace_back(strat.first, btn);

        if (strat.first == strategies::chosen_strategy) {
            btn->set_font(roboto_regular_bold_16);

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

            current_selected_idx = i;
        }

        x += 140;
        if (x + 130 > LV_HOR_RES_MAX - 10) {
            x = 115;
            y += 60;
        }
        i++;
    }
}

void confirm_selection_callback(lv_event_t* e) {
    finished_selection = true;
}

void confirm_selection(void) {
    confirm_button = new renderer::NamedButton(
        "Confirm", 
        roboto_regular_16, 
        20, 175, 70, 50
    );

    lv_obj_add_event_cb(confirm_button->button, confirm_selection_callback, LV_EVENT_CLICKED, NULL);
}

void init_gif(void) {
    miku_gif = lv_gif_create(lv_scr_act());
    lv_obj_set_pos(miku_gif, 380, 10);

    FILE* miku_test = fopen("kaito-miku.gif", "r");
    if (miku_test == NULL) {
        printf("Failed to open miku gif\n");
        return;
    }
    lv_gif_set_src(miku_gif, "S/kaito-miku.gif");
}

void init(void) {
    if (initialized) return;
    initialized = true;

    create_logo();
    team_selector();
    auton_strategy();
    confirm_selection();
    init_gif();
}

void destroy(void) {
    if (!initialized) return;
    initialized = false;

    lv_obj_del(logo);

    delete team_selection_text;
    delete team_button;
    delete side_button;
    delete auton_selection_text;

    for (auto i : auton_buttons) {
        delete i.second;
    }

    delete confirm_button;

    lv_obj_del(selected_auton_box);
    lv_style_reset(selected_auton_box_style);

    lv_obj_del(miku_gif);
}
}