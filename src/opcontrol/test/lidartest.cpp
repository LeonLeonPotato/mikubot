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

    float angle = 0;
    float dist = rand() % 50 + 50;

    while (true) {
        angle += (rand() % 100 + 50) / 1000.0;
        dist += (rand() % 100 - 50) / 2.0;
        dist = std::clamp(dist, 20.0f, 400.0f / sqrtf(2));

        json j = {
            {"angle", angle},
            {"distance", dist}
        };

        std::cout << j.dump() << std::endl;

        pros::delay(50);
    }
}