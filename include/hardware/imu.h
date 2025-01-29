#pragma once

#include "device.h"
#include "mathtils.h"
#include "pros/imu.h"
#include <cmath>

namespace hardware {
class IMUGroup : public AbstractDevice {
    public:
        IMUGroup(const std::vector<int>& ports)
            : AbstractDevice(ports) {}

        void calibrate(bool blocking = true) {
            if (!poll_mutex()) return;

            for (const auto& p : ports) {
                pros::c::imu_reset(p);
            }

            if (blocking) {
                while (true) {
                    bool done = true;
                    for (const auto& p : ports) {
                        done = done && ((pros::c::imu_get_status(p) & pros::E_IMU_STATUS_CALIBRATING) == 0);
                    }
                    if (done) break;
                    pros::c::delay(10);
                }
            }
        }

        float get_heading(void) const {
            if (ports.size() == 0) return 0;
            float ref = pros::c::imu_get_heading(ports[0]);
            std::vector<double> absolutes = {ref};
            for (int i = 1; i < ports.size(); i++) {
                absolutes.push_back(ref + minimum_mod_diff(ref, pros::c::imu_get_heading(ports[i]), 360));
            }

            return modfix(average(absolutes), 360);
        }

        float get_rotation(void) const {
            if (ports.size() == 0) return 0;
            std::vector<double> vec;
            for (int i = 1; i < ports.size(); i++) {
                vec.push_back(pros::c::imu_get_rotation(ports[i]));
            }

            return average(vec);
        }
};

class IMU : public IMUGroup {
    public:
        explicit IMU(int port) : IMUGroup({port}) {}
};
}