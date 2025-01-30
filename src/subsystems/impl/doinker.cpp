#include "subsystems/impl/doinker.h"
#include "essential.h"
#include "config.h"
#include "api.h"

using namespace subsystems;

Doinker* Doinker::instance = nullptr;

void Doinker::tick() {
    if (!poll_mutex()) return;

    bool press = robot::master.get_digital_new_press(config::keybinds::doinker);
    if (press) {
        robot::doinker.toggle();
    }
}