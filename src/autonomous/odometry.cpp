#include "autonomous/odometry.h"
#include "config.h"
#include "essential.h"

#include "api.h"
#include "mathtils.h"

static pros::task_t task = nullptr;

static void run(void* args) {
    long long iterations = 0;
    long long ltime = pros::micros();
    float ls = rad(robot::side_encoder.get_position() / 100.0f);
    float lb = rad(robot::back_encoder.get_position() / 100.0f);
    float ltheta = rad(robot::inertial.get_rotation());

    pros::delay(10);
    while (true) {
        const double dt = (pros::micros() - ltime) / 1e6f;
        ltime = pros::micros();

        robot::theta = rad(robot::inertial.get_rotation());
        const float dtheta = robot::theta - ltheta;
        ltheta = robot::theta;

        robot::angular_acceleration = (dtheta / dt - robot::angular_velocity) / dt;
        robot::angular_velocity = dtheta / dt;

        const float cs = rad(robot::side_encoder.get_position() / 100.0);
        const float cb = rad(robot::back_encoder.get_position() / 100.0);
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
        pros::delay(10);
    }
}

static void simulated_odometry(void* args) {
    const float gain = 10.01f / 1000;
    const float time_constant = 0.120339f;

    float lv = 0, rv = 0;
    long long last_time = pros::micros();
    while (true) {
        float dt = (pros::micros() - last_time) / 1e6f;
        last_time = pros::micros();

        lv += (robot::left_set_voltage * gain - lv) / time_constant * dt;
        rv += (robot::right_set_voltage * gain - rv) / time_constant * dt;

        float v_theta = (lv - rv) * robot::DRIVETRAIN_LINEAR_MULT / robot::DRIVETRAIN_WIDTH;
        float half = v_theta * dt / 2;
        float v_chord = (lv + rv) * robot::DRIVETRAIN_LINEAR_MULT * 0.5f * sinc(half);

        Eigen::Vector2f velocity = {
            v_chord * sinf(half + robot::theta),
            v_chord * cosf(half + robot::theta)
        };
        robot::angular_acceleration = (v_theta - robot::angular_velocity) / dt;
        robot::angular_velocity = v_theta;
        robot::theta += v_theta * dt;
        robot::acceleration = (velocity - robot::velocity) / dt;
        robot::velocity = velocity;
        robot::pos += velocity * dt;

        pros::delay(10);
    }
}

void odometry::start_task() {
    if (task != nullptr) return;
    if (config::ONLY_BRAIN) 
        task = pros::c::task_create(simulated_odometry, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
    else
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