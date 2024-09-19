#include "autonomous/odometry.h"
#include "api.h"
#include "robot.h"

#define rad(x) (x * 0.01745329251f)

namespace odometry {
pros::Task* task;

void run(void* args) {
    float ls = rad(robot::side_encoder.get_position() / 100);
    float lb = rad(robot::side_encoder.get_position() / 100);
    float ltheta = rad(robot::inertial.get_rotation());

    while (true) {
        float ctheta = rad(robot::inertial.get_rotation());
        float dtheta = ctheta - ltheta;
        ltheta = ctheta;

        robot::theta += dtheta;
        if (dtheta < rad(1)) dtheta = 0;

        float cs = rad(robot::side_encoder.get_position() / 100);
        float cb = rad(robot::side_encoder.get_position() / 100);
        float ds = cs - ls;
        float db = cb - lb;
        ls = cs;
        lb = cb;

        float travel_side = ds * robot::TRACKING_WHEEL_RADIUS;
        float travel_back = db * robot::TRACKING_WHEEL_RADIUS;

        if (dtheta != 0) {
            const float ch = 2 * sin(dtheta / 2);
            travel_side = ch * (travel_side / dtheta + robot::SIDE_TRACKING_WHEEL_OFFSET);
            travel_back = ch * (travel_back / dtheta - robot::BACK_TRACKING_WHEEL_OFFSET);
        }

        double av_theta = robot::theta - dtheta / 2;
        robot::x += travel_side * sin(av_theta) - travel_back * cos(av_theta);
        robot::y += travel_side * cos(av_theta) + travel_back * sin(av_theta);

        pros::delay(10);
    }
}

void init(void) {
    task = new pros::Task(run);
}

void start(void) {
    (*task).resume();
}

void stop(void) {
    (*task).suspend();
}
}