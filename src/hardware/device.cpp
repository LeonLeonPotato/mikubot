#include "hardware/device.h"
#include "pros/apix.h"
#include "pros/rtos.h"
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace hardware {
bool AbstractDevice::used_ports[21] = {false};

AbstractDevice::AbstractDevice(const std::vector<int>& ports)
    : ports(ports) {
    char msg[128] = {0};
    for (int i = 0; i < ports.size(); i++) {
        const auto p = abs(ports[i]);
        if (p < 1 || p > 21) {
            sprintf(msg, "Creation of smart port device with invalid port number %d (Expected port between 1-21)", p);
            throw std::invalid_argument(msg);
        }

        if (used_ports[p]) {
            sprintf(msg, "Creation of smart port device with already used port %d", p);
            throw std::invalid_argument(msg);
        }

        used_ports[p] = true;
        mutexes.push_back(pros::c::mutex_create());
    }
}

AbstractDevice::~AbstractDevice() {
    for (int i = 0; i < ports.size(); i++) {
        used_ports[abs(ports[i])] = false;
        pros::c::mutex_delete(mutexes[i]);
    }
}

bool AbstractDevice::acquire_mutex(uint32_t timeout) {
    bool ret = true;
    const auto cur_task = pros::c::task_get_current();
    for (const auto& m : mutexes) {
        if (pros::c::mutex_get_owner(m) != cur_task)
            ret = ret && pros::c::mutex_take(m, timeout);
        if (!ret) return false;
    }
    return ret;
}

bool AbstractDevice::poll_mutex(void) const {
    bool ret = true;
    const auto cur_task = pros::c::task_get_current();
    for (const auto& m : mutexes) {
        ret = ret && (pros::c::mutex_get_owner(m) == cur_task);
        if (!ret) return false;
    }
    return ret;
}

void AbstractDevice::release_mutex() {
    for (const auto& m : mutexes) {
        pros::c::mutex_give(m);
    }
}
}