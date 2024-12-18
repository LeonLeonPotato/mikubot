#include "opcontrol/test/odom_center.h"
#include "essential.h"
#include <vector>

static int rotations = 10;
static int max_time = 1e3;

void controls::odom_centering::tick() {
    printf("Odom centering tick not implemented\n");
}

void controls::odom_centering::run() {
    float start_angle = robot::theta;
    robot::volt(1.0f, -1.0f);

    std::vector<Eigen::Vector2f> points;
    points.reserve(10000);
    int start_t = pros::millis();

    while (fabsf(start_angle - robot::theta) < rotations*M_TWOPI && pros::millis() - start_t < max_time) {
        pros::delay(10);
        points.push_back(robot::pos);
    }

    robot::brake();

    std::cout << "Points: " << points.size() << "\n";
    std::cout << "P=\\left[";
    for (int i = 0; i < points.size(); i++) {
        const auto& p = points[i];
        char buf[64];
        int n = sprintf(buf, "\\left(%.3f,\\ %.3f\\right)", p(0), p(1));
        if (i != points.size() - 1) sprintf(buf + n, ", ");
        std::cout << buf;
        pros::delay(20);
    }
    std::cout << "\\right]" << "\n";
}

void controls::odom_centering::start_task() {
    printf("Odom centering start task not implemented\n");
}

void controls::odom_centering::pause() {
    printf("Odom centering pause not implemented\n");
}

void controls::odom_centering::resume() {
    printf("Odom centering resume not implemented\n");
}

void controls::odom_centering::stop_task() {
    printf("Odom centering stop task not implemented\n");
}