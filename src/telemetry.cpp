#include "telemetry.h"
#include "essential.h"
#include "pros/misc.hpp"
#include <numeric>
#include <queue>

using namespace telemetry;

volatile int telemetry::delay = 10;

static pros::task_t task = nullptr;
static volatile int mode = TO_STDOUT;
static FILE* file = nullptr;
static std::string filename;
static std::queue<std::string> log_queue;

static void initialize_log_file(void) {
    if (!pros::usd::is_installed()) {
        printf("[Telemetry] SD card not installed\n");
        return;
    }

    FILE* numfile = fopen("/usd/lognum.txt", "r");
    int num = 0;
    if (numfile) {
        fscanf(numfile, "%d", &num);
        fclose(numfile);
    }
    printf("[Telemetry] Log number: %d\n", num);

    numfile = fopen("/usd/lognum.txt", "w");
    if (!numfile) return;
    fprintf(numfile, "%d", num + 1);
    fclose(numfile);

    filename = "/usd/log" + std::to_string(num) + ".txt";
    file = fopen(filename.c_str(), "w");
}

template <typename T>
static float average(const std::vector<T>& v) {
    if (v.empty()) return 0;
    return std::reduce(v.begin(), v.end()) / (float) v.size();
}

static void logging_task(void* args) {
    bool will_log_file = (mode & TO_FILE) >> 1;

    if (!file || !pros::usd::is_installed()) {
        printf("[Telemetry] Log file could not be opened\n");
        will_log_file = false;
    } else {
        printf("[Telemetry] Log file opened\n");
        printf("[Telemetry] Log file: %s\n", filename.c_str());

        fprintf(file, "time,x,y,theta,left_velocity,right_velocity,left_voltage,right_voltage,left_actual_velocity,right_actual_velocity,left_actual_voltage,right_actual_voltage,clamp,intake,conveyor\n");
        fflush(file);
    }

    int last_dump_time = pros::millis();
    while (true) {
        char buffer[256]; memset(buffer, 0, 256);
        sprintf(buffer, "%lld,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%f,%f\n",
            pros::micros(),
            robot::pos.x(), robot::pos.y(), robot::theta,
            robot::left_set_velocity, robot::right_set_velocity,
            robot::left_set_voltage, robot::right_set_voltage,
            average(robot::left_motors.get_actual_velocity_all()),
            average(robot::right_motors.get_actual_velocity_all()),
            average(robot::left_motors.get_voltage_all()),
            average(robot::right_motors.get_voltage_all()),
            (int) robot::clamp.is_extended(), 
            robot::intake.get_actual_velocity(), 
            robot::conveyor.get_actual_velocity()
        );

        if (mode & TO_STDOUT) {
            printf("%s", buffer);
        }

        if (will_log_file) {
            log_queue.push(std::string(buffer));

            if (pros::millis() - last_dump_time > 250) {
                last_dump_time = pros::millis();
                while (!log_queue.empty()) {
                    fprintf(file, "%s", log_queue.front().c_str());
                    log_queue.pop();
                }

                fflush(file);
            }
        }

        pros::delay(delay);
    }
}

void telemetry::set_mode(int m) {
    mode = m;
}

void telemetry::start_task(void) {
    if (file == nullptr) initialize_log_file();

    if (task == nullptr) {
        task = pros::c::task_create(logging_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Telemetry");
    }
}

void telemetry::stop_task(void) {
    if (task != nullptr) {
        pros::c::task_delete(task);
        task = nullptr;
    }
}

void telemetry::pause(void) {
    if (task != nullptr) {
        pros::c::task_suspend(task);
    }
}

void telemetry::resume(void) {
    if (task != nullptr) {
        pros::c::task_resume(task);
    }
}