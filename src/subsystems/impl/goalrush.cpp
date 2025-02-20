#include "subsystems/impl/goalrush.h"
#include "essential.h"
#include "config.h"
#include "api.h"

using namespace subsystems;

Goalrush* Goalrush::instance = nullptr;

void Goalrush::tick() {
    if (!poll_mutex()) return;

    bool press = robot::master.get_digital_new_press(config::keybinds::doinker);
    if (press) {
        robot::doinker.toggle();
    }
}