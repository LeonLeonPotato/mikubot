#pragma once

namespace controls::wallmech {
enum class State {
    RESTING = 0,
    PRIMED = 1,
    FIRING = 2,
    OVERRIDE = 3
};

void start_api_task();
void stop_api_task();
void exposed_go_to_state(State state);

void run(void);
void tick(void);
void start_task(void);
void pause(void);
void resume(void);
void stop_task(void);
} // namespace controls::wallmech