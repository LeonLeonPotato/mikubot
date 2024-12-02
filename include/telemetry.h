#pragma once

namespace telemetry {
extern volatile int delay;

void set_mode(int m);
void start_task(void);
void stop_task(void);
void pause(void);
void resume(void);
};