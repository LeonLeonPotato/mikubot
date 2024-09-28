#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include <string>
#include "liblvgl/lvgl.h"

namespace renderer {

class LVGLObject {
    public:
        virtual void move(int x, int y) const = 0;
        virtual void resize(int w, int h) const = 0;
};

class Text : public LVGLObject {
    public:
        lv_obj_t* text;
        lv_style_t* text_style;

        Text(const std::string& text, const lv_font_t& font, int x, int y, 
            int color = 0xFFFFFF, lv_obj_t* canvas = 0) 
        {
            if (reinterpret_cast<int>(canvas) == 0) canvas = lv_scr_act();
            this->text = lv_label_create(canvas);
            lv_label_set_text(this->text, text.c_str());
            lv_obj_set_pos(this->text, x, y);

            text_style = new lv_style_t();
            lv_style_init(text_style);
            lv_style_set_text_font(text_style, &font);
            lv_obj_add_style(this->text, text_style, 0);
        }
        ~Text(void) {
            lv_obj_del(text);
            lv_style_reset(text_style);
        }

        void rename(const std::string& text) const;
        void set_font(const lv_font_t& font) const;
        void move(int x, int y) const override;
        void resize(int w, int h) const override;
};

class NamedButton : LVGLObject {
    public:
        lv_obj_t* button;
        lv_style_t* button_style;
        lv_obj_t* text;
        lv_style_t* text_style;
        
        NamedButton(const std::string& text, const lv_font_t& font, 
                    int x, int y, int w, int h, int bg_color=0x34aeeb, int text_color=0xFFFFFF, lv_obj_t* canvas = 0) 
        {
            if (reinterpret_cast<int>(canvas) == 0) canvas = lv_scr_act();
            button = lv_btn_create(canvas);
            lv_obj_set_pos(button, x, y);
            lv_obj_set_size(button, w, h);

            button_style = new lv_style_t();
            lv_style_init(button_style);
            lv_style_set_bg_color(button_style, lv_color_hex(bg_color));
            lv_obj_add_style(button, button_style, 0);

            this->text = lv_label_create(button);
            lv_label_set_text(this->text, text.c_str());
            lv_obj_center(this->text);

            text_style = new lv_style_t();
            lv_style_init(text_style);
            lv_style_set_text_color(text_style, lv_color_hex(text_color));
            lv_style_set_text_font(text_style, &font);
            lv_obj_add_style(this->text, text_style, 0);
        }
        ~NamedButton(void) {
            lv_obj_del(button);
            lv_style_reset(button_style);
            lv_style_reset(text_style);
        }

        void rename(const std::string& text) const;
        void set_font(const lv_font_t& font) const;
        void bg_color(int color) const;
        void text_color(int color) const;
        void move(int x, int y) const override;
        void resize(int w, int h) const override;
};

void init(void);
void destroy(void);
}