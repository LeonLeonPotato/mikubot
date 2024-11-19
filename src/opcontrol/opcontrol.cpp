#include "opcontrol/opcontrol.h"
#include "essential.h"

const std::vector<std::function<void(void)>> controls::ticks = {
    driving::tick,
    intake::tick,
    conveyor::tick
};

const std::vector<std::function<void(void)>> controls::runs = {
    driving::run,
    intake::run,
    conveyor::run
};

const std::vector<std::function<void(void)>> controls::start_tasks = {
    driving::start_task,
    intake::start_task,
    conveyor::start_task
};

const std::vector<std::function<void(void)>> controls::stop_tasks = {
    driving::stop_task,
    intake::stop_task,
    conveyor::stop_task
};