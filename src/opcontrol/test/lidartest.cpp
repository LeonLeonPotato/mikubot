#include "opcontrol/test/lidartest.h"
#include "essential.h"
#include "api.h"

#include "nlohmann/json.h"

#include <random>
#include <iostream>

using namespace controls;

using json = nlohmann::json;

void lidartest::run(void) {
    std::cout << "Lidar test started" << std::endl;

    pros::IMU inertial(12);
    inertial.reset(true);

    pros::Distance lidar(11);

    while (true) {
        json j;
        j["distance"] = lidar.get();
        j["confidence"] = lidar.get_confidence();
        float pitch = inertial.get_pitch() / 180.0f * M_PI;
        float yaw = inertial.get_yaw() / 180.0f * M_PI;
        j["pitch"] = pitch;
        j["yaw"] = yaw;
        printf("LIDAR: %s\n", j.dump().c_str());
        pros::delay(100);
    }
}