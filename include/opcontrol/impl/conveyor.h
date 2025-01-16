#pragma once

namespace controls::conveyor {
void start_api_task();
void stop_api_task();
void exposed_desired_volt(float volt);
void exposed_set_color_sort(bool val);
bool exposed_get_color_sort(void);

void run(void);
void tick(void);
void start_task(void);
void pause(void);
void resume(void);
void stop_task(void);
} // namespace controls::conveyor