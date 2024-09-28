#include "gui/utils.h"

#include "liblvgl/lvgl.h"

#include "string"
namespace renderer {
struct NamedButton {
    lv_obj_t* button;
    lv_style_t* button_style;
    lv_obj_t* text;
    lv_style_t* text_style;
};

NamedButton* create_button(std::string& text, lv_font_t& font, int x, int y, int w, int h, int bg_color, int text_color) {
    NamedButton* button = new NamedButton();
    button->button = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(button->button, x, y);
    lv_obj_set_size(button->button, w, h);

    button->button_style = new lv_style_t();
    lv_style_init(button->button_style);
    lv_style_set_bg_color(button->button_style, lv_color_hex(bg_color));
    lv_obj_add_style(button->button, button->button_style, 0);

    button->text = lv_label_create(button->button);
    lv_label_set_text(button->text, text.c_str());
    lv_obj_center(button->text);

    button->text_style = new lv_style_t();
    lv_style_init(button->text_style);
    lv_style_set_text_color(button->text_style, lv_color_hex(text_color));
    lv_style_set_text_font(button->text_style, &font);
    lv_obj_add_style(button->text, button->text_style, 0);

    return button;
}

void init(void) {
}

void destroy(void) {

}
}
