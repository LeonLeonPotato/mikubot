#include "opcontrol/test/lidartest.h"
#include "essential.h"
#include "api.h"

#include <iostream>

using namespace controls;

void lidartest::run(void) {
    std::cout << "Lidar test started" << std::endl;
    while (true) {
        pros::delay(20);
    }
}