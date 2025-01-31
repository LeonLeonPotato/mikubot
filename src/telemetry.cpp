#include "telemetry.h"
#include "essential.h"
#include "pros/misc.hpp"
#include <numeric>
#include <queue>

using namespace telemetry;

volatile int telemetry::delay = 25;

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

static void logging_task(void* args) {
    bool will_log_file = (mode & TO_FILE) >> 1;

    if (!file || !pros::usd::is_installed()) {
        printf("[Telemetry] Log file could not be opened\n");
        will_log_file = false;
    } else {
        printf("[Telemetry] Log file opened\n");
        printf("[Telemetry] Log file: %s\n", filename.c_str());

        fprintf(file, "time,x,y,theta\n");
        fflush(file);
    }

    int last_dump_time = pros::millis();
    while (true) {
        char buffer[256] = {0};
        sprintf(buffer, "%lld,%f,%f,%f\n",
            pros::micros(),
            robot::x(), robot::y(), robot::theta()
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