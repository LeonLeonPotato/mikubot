#include "gui/goofymiku.h"
#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include <cstdio>
#include "api.h" // IWYU pragma: keep
#include "gui/utils.h" // IWYU pragma: keep

static bool initialized = false;
static lv_obj_t* miku = nullptr;

static void run() {
    if (!renderer::check_exists("/", "funny.gif")) {
        printf("[Funny] Funny gif not found, check SD card\n");
        return;
    }

    miku = lv_gif_create(lv_scr_act());
    lv_obj_set_pos(miku, 156, 0);
    lv_gif_set_src(miku, "S/funny.gif");
}

void opcontrolfun::init(void) {
    if (initialized) return;
    initialized = true;

    run();
}

void opcontrolfun::destroy(void) {
    if (!initialized) return;
    initialized = false;

    if (miku) lv_obj_del(miku);
}