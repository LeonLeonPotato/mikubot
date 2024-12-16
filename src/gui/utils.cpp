#include "gui/utils.h"
#include "api.h"
#include "pros/misc.hpp"

using namespace renderer;

void Text::rename(const std::string& text) const {
    lv_label_set_text(this->text, text.c_str());
    lv_obj_center(this->text);
}

void Text::set_font(const lv_font_t& font) const {
    lv_style_set_text_font(this->text_style, &font);
}

void Text::move(int x, int y) const {
    lv_obj_set_pos(this->text, x, y);
}

void Text::resize(int w, int h) const {
    lv_obj_set_size(this->text, w, h);
}

void NamedButton::rename(const std::string& text) const {
    lv_label_set_text(this->text, text.c_str());
    lv_obj_center(this->text);
}

void NamedButton::set_font(const lv_font_t& font) const {
    lv_style_set_text_font(this->text_style, &font);
}

void NamedButton::bg_color(int color) const {
    lv_style_set_bg_color(this->button_style, lv_color_hex(color));
}

void NamedButton::text_color(int color) const {
    lv_style_set_text_color(this->text_style, lv_color_hex(color));
}

void NamedButton::move(int x, int y) const {
    lv_obj_set_pos(this->button, x, y);
}

void NamedButton::resize(int w, int h) const {
    lv_obj_set_size(this->button, w, h);
}

bool renderer::check_exists(const char* path, const char* name) {
    if (!pros::usd::is_installed()) return false;
    char buffer[4096]; 
    memset(buffer, 0, sizeof(buffer));
    pros::usd::list_files(path, buffer, sizeof(buffer));
    return strstr(buffer, name) != nullptr;
}

void renderer::init(void) {
}

void renderer::destroy(void) {
}
