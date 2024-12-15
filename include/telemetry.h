#pragma once

namespace telemetry {
constexpr int TO_FILE = 0b10;
constexpr int TO_STDOUT = 0b01;

extern volatile int delay;

void set_mode(int m);
void start_task(void);
void stop_task(void);
void pause(void);
void resume(void);
};