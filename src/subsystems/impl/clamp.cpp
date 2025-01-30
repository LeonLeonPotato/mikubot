#include "subsystems/impl/clamp.h"
#include "essential.h"
#include "config.h"

using namespace subsystems;

void Clamp::tick(void) {
    if (!poll_mutex()) return;

    bool press = robot::master.get_digital_new_press(config::keybinds::clamp);
    if (press) {
        robot::clamp.toggle();
    }
}
