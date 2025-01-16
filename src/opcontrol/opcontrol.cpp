#include "opcontrol/opcontrol.h"
#include "essential.h"

// TODO: OOP this
// TODO: When OOPing this, add an API section

const std::vector<std::function<void(void)>> controls::ticks = {
    driving::tick,
    intake::tick,
    conveyor::tick,
    clamp::tick,
    wallmech::tick,
    doinker::tick,
};

const std::vector<std::function<void(void)>> controls::runs = {
    driving::run,
    intake::run,
    conveyor::run,
    clamp::run,
    wallmech::run,
    doinker::run,
    // odom_centering::run
};

const std::vector<std::function<void(void)>> controls::start_tasks = {
    driving::start_task,
    intake::start_task,
    conveyor::start_task,
    clamp::start_task,
    wallmech::start_task,
    doinker::start_task,
};

const std::vector<std::function<void(void)>> controls::stop_tasks = {
    driving::stop_task,
    intake::stop_task,
    conveyor::stop_task,
    clamp::stop_task,
    wallmech::stop_task,
    doinker::stop_task
};