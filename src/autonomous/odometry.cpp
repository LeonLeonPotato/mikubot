#include "autonomous/odometry.h"
#include "essential.h"

#include "api.h"

#define rad(x) (x * 0.01745329251f)

static pros::task_t task = nullptr;

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

    pros::delay(5);
    while (true) {
        const double dt = (pros::micros() - ltime) / 1000000.0f;
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
            const float cs = rad(robot::side_encoder.get_position() / 100.0);
            const float cb = rad(robot::back_encoder.get_position() / 100.0);
        #else
            const float cs = 0;
            const float cb = 0;
        #endif
        float travel_side = (cs - ls) * robot::TRACKING_WHEEL_RADIUS;
        float travel_back = (cb - lb) * robot::TRACKING_WHEEL_RADIUS;
        ls = cs;
        lb = cb;

        if (dtheta != 0) {
            const float ch = 2 * sin(dtheta / 2);
            travel_side = ch * (travel_side / dtheta + robot::SIDE_TRACKING_WHEEL_OFFSET);
            travel_back = ch * (travel_back / dtheta + robot::BACK_TRACKING_WHEEL_OFFSET);
        }

        const float av_theta = robot::theta - dtheta / 2;

        const Eigen::Vector2f last_velocity = robot::velocity;
        const Eigen::Vector2f travel = {
            travel_side * sinf(av_theta) - travel_back * cosf(av_theta),
            travel_side * cosf(av_theta) + travel_back * sinf(av_theta)
        };

        robot::pos += travel;

        robot::velocity = travel / dt;
        robot::acceleration = (robot::velocity - last_velocity) / dt;

        iterations++;
        pros::delay(5);
    }
}

void odometry::start_task() {
    if (task != nullptr) return;
    task = pros::c::task_create(run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
}

void odometry::pause() {
    if (task == nullptr) return;
    pros::c::task_suspend(task);
}

void odometry::resume() {
    if (task == nullptr) return;
    pros::c::task_resume(task);
}

void odometry::stop_task() {
    if (task == nullptr) return;
    pros::c::task_delete(task);
    task = nullptr;
}