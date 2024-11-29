#include "autonomous/odometry.h"
#include "essential.h"

#include "api.h"

#define rad(x) (x * 0.01745329251f)

namespace odometry {
pros::task_t task;

void run(void* args) {
    long long iterations = 0;
    long long ltime = pros::micros();
    #ifndef MIKU_TESTENV
        float ls = rad(robot::side_encoder.get_position() / 100.0f);
        float lb = rad(robot::back_encoder.get_position() / 100.0f);
        float ltheta = rad(robot::inertial.get_rotation());
    #else
        float ls = 0;
        float lb = 0;
        float ltheta = 0;
    #endif

    while (true) {
        const long long dt = (pros::micros() - ltime) / 1000000.0f;
        ltime = pros::micros();

        #ifndef MIKU_TESTENV
            robot::theta = rad(robot::inertial.get_rotation());
        #else
            robot::theta = 0;
        #endif
        const float dtheta = robot::theta - ltheta;
        ltheta = robot::theta;

        robot::angular_acceleration = (dtheta / dt - robot::angular_velocity) / dt;
        robot::angular_velocity = dtheta / dt;

        #ifndef MIKU_TESTENV
            float cs = rad(robot::side_encoder.get_position() / 100.0);
            float cb = rad(robot::back_encoder.get_position() / 100.0);
        #else
            float cs = 0;
            float cb = 0;
        #endif
        float travel_side = (cs - ls) * robot::TRACKING_WHEEL_RADIUS;
        float travel_back = (cb - lb) * robot::TRACKING_WHEEL_RADIUS;
        ls = cs;
        lb = cb;

        if (dtheta != 0) {
            float ch = 2 * sin(dtheta / 2);
            // ITS MINUS AND PLUS COMBINATION!!! DO NOT CHANGE!!!!
            travel_side = ch * (travel_side / dtheta - robot::SIDE_TRACKING_WHEEL_OFFSET);
            travel_back = ch * (travel_back / dtheta + robot::BACK_TRACKING_WHEEL_OFFSET);
        }

        const float av_theta = robot::theta - dtheta / 2;

        Eigen::Vector2f last_velocity = robot::velocity;
        Eigen::Vector2f travel = {
            -travel_side * cos(av_theta) - travel_back * sin(av_theta),
            travel_side * sin(av_theta) - travel_back * cos(av_theta)
        };

        robot::pos += travel;

        robot::velocity = travel / dt;
        robot::acceleration = (robot::velocity - last_velocity) / dt;

        iterations++;
        pros::delay(5);
    }
}

void init(void) {
    #ifndef MIKU_TESTENV
        task = pros::c::task_create(run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
    #endif
}

void start(void) {
    #ifndef MIKU_TESTENV
        pros::c::task_resume(task);
    #endif
}

void stop(void) {
    #ifndef MIKU_TESTENV
        pros::c::task_suspend(task);
    #endif
}
}