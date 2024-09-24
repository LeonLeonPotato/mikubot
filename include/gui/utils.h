#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "liblvgl/lvgl.h"

namespace renderer {
    void init(void);
    void destroy(void);
}