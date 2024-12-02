#include "telemetry.h"
#include "essential.h"
#include <numeric>
#include <queue>

using namespace telemetry;

volatile int telemetry::delay = 10;

static pros::task_t task = nullptr;
static volatile int mode = 0b10;
static FILE* file = nullptr;
static std::string filename;
static std::queue<std::string> log_queue;

static void initialize_log_file(void) {
    FILE* numfile = fopen("/usd/lognum.txt", "r");
    int num = 0;
    if (numfile) {
        fscanf(numfile, "%d", &num);
        fclose(numfile);
    }
    printf("[Telemetry] Log number: %d\n", num);

    numfile = fopen("/usd/lognum.txt", "w");
    if (numfile == nullptr) return;
    fprintf(numfile, "%d", num + 1);
    fclose(numfile);

    filename = "/usd/log" + std::to_string(num) + ".txt";
    file = fopen(filename.c_str(), "w");
}

template <typename T>
static T average(const std::vector<T>& v) {
    if (v.empty()) return 0;

    auto const count = static_cast<float>(v.size());
    return std::reduce(v.begin(), v.end()) / count;
}

static void logging_task(void* args) {
    #ifndef MIKU_TESTENV
    initialize_log_file();
    bool will_log_file = (bool) (mode & 0b10) >> 1;

    if (file == nullptr) {
        printf("[Telemetry] Log file could not be opened\n");
        will_log_file = false;
    } else {
        printf("[Telemetry] Log file opened\n");
        printf("[Telemetry] Log file: %s\n", filename.c_str());
    }

    int last_dump_time = pros::millis();
    while (true) {
        char buffer[256]; memset(buffer, 0, 256);
        sprintf(buffer, "%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f\n",
            robot::pos.x(), robot::pos.y(), robot::theta,
            robot::left_set_voltage, robot::right_set_voltage,
            robot::left_set_velocity, robot::right_set_velocity,
            average(robot::left_motors.get_actual_velocity_all()),
            average(robot::right_motors.get_actual_velocity_all()),
            average(robot::left_motors.get_voltage_all()),
            average(robot::right_motors.get_voltage_all())
        );

        if ((bool) (mode & 0b01) >> 0) {
            printf("%s", buffer);
        }

        if (will_log_file) {
            log_queue.push(std::string(buffer));
            if (pros::millis() - last_dump_time > 500) {
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
    #endif
}

void telemetry::set_mode(int m) {
    mode = m;
}

void telemetry::start_task(void) {
    #ifndef MIKU_TESTENV
    if (task == nullptr) {
        task = pros::c::task_create(logging_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Telemetry");
    }
    #endif
}

void telemetry::stop_task(void) {
    #ifndef MIKU_TESTENV
    if (task != nullptr) {
        pros::c::task_delete(task);
        task = nullptr;
    }
    #endif
}

void telemetry::pause(void) {
    #ifndef MIKU_TESTENV
    if (task != nullptr) {
        pros::c::task_suspend(task);
    }
    #endif
}

void telemetry::resume(void) {
    #ifndef MIKU_TESTENV
    if (task != nullptr) {
        pros::c::task_resume(task);
    }
    #endif
}