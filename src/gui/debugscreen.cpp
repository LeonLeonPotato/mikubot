#include "gui/debugscreen.h"
#include "essential.h"
#include "gui/utils.h"
#include "liblvgl/lv_api_map.h"
#include "liblvgl/lvgl.h"
#include "librsc/fonts.hpp"
#include "pros/rtos.h"
#include <cstddef>

static bool initialized = false;
std::string debugscreen::debug_message = "Debug message";
static renderer::Text* text;
static pros::task_t update_text_task;

static void update_text_task_func(void* p) {
    char st[512]; memset(st, 0, sizeof(st));
    while (true) {
        sprintf(st, "Robot pos: [%.2f, %.2f]   Robot angle: %.2f\n%s", 
            robot::pos.x(), robot::pos.y(), robot::theta, debugscreen::debug_message.c_str());
        text->rename(st);
    }
}

void debugscreen::init() {
    if (initialized) return; initialized = true;
    text = new renderer::Text(debug_message.c_str(), fonts::jetbrains_mono_regular_24, 0, 0, 0xFFFFFF);
    update_text_task = pros::c::task_create(update_text_task_func, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Update debug text");
}

void debugscreen::destroy() {
    if (!initialized) return; initialized = false;
    delete text;
    pros::c::task_delete(update_text_task);
    update_text_task = nullptr;
}