#include "gui/goofymiku.h"
#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include <cstdio>
#include "api.h" // IWYU pragma: keep

static bool initialized = false;
static lv_obj_t* miku = nullptr;

static void run() {
    #ifndef MIKU_TESTENV
    FILE* file = fopen("/usd/funny.gif", "r");
    if (!file) {
        printf("[Funny] Failed to open miku gif. Is the SD Card installed?\n");
        return;
    }
    fclose(file);
    pros::delay(10);

    miku = lv_gif_create(lv_scr_act());
    lv_obj_set_pos(miku, 156, 0);
    lv_gif_set_src(miku, "S/funny.gif");
    #endif
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