#include "opcontrol/test/lidartest.h"
#include "autonomous/controllers/pid.h"
#include "essential.h"
#include "api.h"

#include "nlohmann/json.h"

#include <random>
#include <iostream>

using namespace controls;

using json = nlohmann::json;

void lidartest::run(void) {
    std::cout << "Lidar test started" << std::endl;

    pros::Motor updown(18);

    pros::vision_signature_s_t goal = pros::Vision::signature_from_utility(
        1, -1885, -1271, -1578, 12189, 13607, 12898, 3.000, 0
    );

    pros::Vision vision(16);
    vision.set_exposure(100);
    vision.set_signature(1, &goal);

    controllers::PID pid(20, 0.0, -0.01, 999, 999, true);

    while (true) {
        auto obj = vision.get_by_sig(0, 1);
        if (obj.signature == 255) {
            printf("Object not found\n");
            updown.move_velocity(0);
            pros::delay(5);
            pid.reset();
            continue;
        }

        printf("Object found: (Sig=%d, %d, %d) | ", obj.signature, (int) obj.x_middle_coord, (int) (obj.y_middle_coord));

        float error = obj.y_middle_coord - 158;
        int control = (int) pid.get(error);
        printf("Control: %d | Error: %f\n", control, error);

        updown.move_voltage(control);

        pros::delay(5);
    }
}