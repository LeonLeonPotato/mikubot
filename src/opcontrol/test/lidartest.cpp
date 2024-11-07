#include "opcontrol/test/lidartest.h"
#include "essential.h"
#include "api.h"

#include "nlohmann/json.h"

#include <random>
#include <iostream>

using namespace controls;

using json = nlohmann::json;

pros::quaternion_s multiply(pros::quaternion_s& one, pros::quaternion_s& two) {
    pros::quaternion_s result;
    result.w = one.w * two.w - one.x * two.x - one.y * two.y - one.z * two.z;
    result.x = one.w * two.x + one.x * two.w + one.y * two.z - one.z * two.y;
    result.y = one.w * two.y - one.x * two.z + one.y * two.w + one.z * two.x;
    result.z = one.w * two.z + one.x * two.y - one.y * two.x + one.z * two.w;
    return result;
}

pros::quaternion_s invert(pros::quaternion_s& quat) {
    pros::quaternion_s result;
    const double norm = (quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    result.w = quat.w / norm;
    result.x = -quat.x / norm;
    result.y = -quat.y / norm;
    result.z = -quat.z / norm;
    return result;
}

void lidartest::run(void) {
    double vx = 0, vy = 0, vz = 0;
    double x = 0, y = 0, z = 0;

    std::cout << "Lidar test started" << std::endl;

    pros::IMU inertial(12);
    inertial.reset(true);
    auto __g = inertial.get_accel();
    auto q_g_accel = pros::quaternion_s {__g.x, __g.y, __g.z, 0};

    // pros::adi::Ultrasonic lidar('A', 'B');
    pros::Distance lidar(11);

    long long last = pros::micros();    

    while (true) {
        pros::delay(10);

        // double dt = (double) (pros::micros() - last) / 1e6;
        // last = pros::micros();
        // auto accel = inertial.get_accel();

        // pros::quaternion_s q = inertial.get_quaternion();
        // pros::quaternion_s q_inv = invert(q);
        // auto cur_g = multiply(q_g_accel, q);
        // cur_g = multiply(q_inv, cur_g);
        // accel.x -= cur_g.x;
        // accel.y -= cur_g.y;
        // accel.z -= cur_g.z;

        // vx += accel.x * 9.80665 * dt;
        // vy += accel.y * 9.80665 * dt;
        // vz += accel.z * 9.80665 * dt;

        // x += vx * dt;
        // y += vy * dt;
        // z += vz * dt; 

        auto q = inertial.get_quaternion();
        double w = q.w;
        double x = q.x;
        double y = q.y;
        double z = q.z;

        json j;
        j["distance"] = lidar.get();
        j["confidence"] = lidar.get_confidence();
        j["quaternion"] = {x, y, z, w};
        printf("LIDAR: %s\n", j.dump().c_str());
    }
}