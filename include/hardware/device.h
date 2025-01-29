#pragma once

#include "pros/rtos.h"
#include <vector>

namespace hardware {
class AbstractDevice {
    private:
        static bool used_ports[21];

    protected:
        std::vector<int> ports;
        std::vector<pros::mutex_t> mutexes;
    
    public:
        AbstractDevice(const std::vector<int>& ports);
        virtual ~AbstractDevice();
        bool acquire_mutex(uint32_t timeout = TIMEOUT_MAX);
        bool poll_mutex(void) const;
        void release_mutex(void);

        const std::vector<int>& get_ports(void) const { return ports; }
        const std::vector<pros::mutex_t>& get_mutexes(void) const { return mutexes; }

};
};