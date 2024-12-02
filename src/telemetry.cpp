#include "telemetry.h"
#include "essential.h"
#include <numeric>

using namespace telemetry;

volatile int telemetry::delay = 10;

static pros::task_t task = nullptr;
static volatile int mode = 0b01 | 0b10;
static FILE* file = nullptr;
static std::string filename;

static void get_log_file(void) {
    FILE* numfile = fopen("/usd/lognum.txt", "r");
    int num = 0;
    if (numfile != nullptr) {
        fscanf(numfile, "%d", &num);
        fclose(numfile);
    }

    numfile = fopen("/usd/lognum.txt", "w");
    if (numfile == nullptr) return;

    fprintf(numfile, "%d", num + 1);
    fclose(numfile);

    filename = "/usd/log" + std::to_string(num) + ".txt";
    file = fopen(filename.c_str(), "w");
}

template <typename T>
static T average(const std::vector<T>& v){
    if (v.empty()) return 0;

    auto const count = static_cast<float>(v.size());
    return std::reduce(v.begin(), v.end()) / count;
}

static void logging_task(void* args) {
    #ifndef MIKU_TESTENV
    get_log_file();
    if (file == nullptr) {
        printf("[Telemetry] Log file could not be opened\n");
        return;
    }
    printf("[Telemetry] Log file opened\n");

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

        if ((bool) (mode & 0b10) >> 1) {
            fprintf(file, "%s", buffer);
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