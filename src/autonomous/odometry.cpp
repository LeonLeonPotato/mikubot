#include "autonomous/odometry.h"
#include "essential.h"

#include "api.h"

#define rad(x) (x * 0.01745329251f)

namespace odometry {
pros::task_t task;

void run(void* args) {
    long long iterations = 0;
    float ls = rad(robot::side_encoder.get_position() / 100);
    float lb = rad(robot::back_encoder.get_position() / 100);
    float ltheta = rad(robot::inertial.get_rotation());
    auto ltime = pros::micros();
    float lx = 0, ly = 0;
    float lvx = 0, lvy = 0;

    while (true) {
        auto dt = (pros::micros() - ltime) / 1000000.0f;
        ltime = pros::micros();

        float ctheta = rad(robot::inertial.get_rotation());
        float dtheta = ctheta - ltheta;
        ltheta = ctheta;
        if (fabs(dtheta) < rad(1)) dtheta = 0;

        robot::theta += dtheta;
        robot::angular_acceleration = (dtheta / dt - robot::angular_velocity) / dt;
        robot::angular_velocity = dtheta / dt;

        float cs = rad(robot::side_encoder.get_position() / 100.0);
        float cb = rad(robot::back_encoder.get_position() / 100.0);
        float ds = cs - ls;
        float db = cb - lb;
        ls = cs;
        lb = cb;
        
        float travel_side = ds * robot::TRACKING_WHEEL_RADIUS;
        float travel_back = db * robot::TRACKING_WHEEL_RADIUS;

        if (dtheta != 0) {
            float ch = 2 * sin(dtheta / 2);
            travel_side = ch * (travel_side / dtheta + robot::SIDE_TRACKING_WHEEL_OFFSET);
            travel_back = ch * (travel_back / dtheta - robot::BACK_TRACKING_WHEEL_OFFSET);
        }

        float av_theta = robot::theta - dtheta / 2;

        lx = robot::x;
        ly = robot::y;
        lvx = robot::velocity_x;
        lvy = robot::velocity_y;

        robot::x += travel_side * cos(av_theta) + travel_back * sin(av_theta);
        robot::y += travel_side * sin(av_theta) - travel_back * cos(av_theta);

        robot::velocity_x = (robot::x - lx) / dt;
        robot::velocity_y = (robot::y - ly) / dt;
        robot::acceleration_x = (robot::velocity_x - lvx) / dt;
        robot::acceleration_y = (robot::velocity_y - lvy) / dt;

        pros::c::delay(10);

        iterations++;
    }
}

void init(void) {
    task = pros::c::task_create(run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void start(void) {
    pros::c::task_suspend(task);
}

void stop(void) {
    pros::c::task_resume(task);
}
}