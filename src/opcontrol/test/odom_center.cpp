#include "opcontrol/test/odom_center.h"
#include "essential.h"
#include "config.h"
#include <vector>

static int rotations = 10;
static int max_time = 5e3;
static int iterations = 5;
static float speed = 0.2f;

static bool last = false;
static bool toggle = false;

void controls::odom_centering::tick() {
    float start_angle = robot::theta;
    robot::velo(speed, -speed);

    Eigen::MatrixX2f points(max_time / 20 + 10, 2);

    Eigen::Vector2f tare = robot::pos;
    float theta = robot::theta;

    int start_t = pros::millis();
    int i = 0;

    while (fabsf(start_angle - robot::theta) < rotations*M_TWOPI && pros::millis() - start_t < max_time) {
        pros::delay(20);
        points.row(i++) = Eigen::Rotation2Df(-theta) * (robot::pos - tare);
    }

    robot::brake();

    std::cout << "[Odom Centering] Points: " << i << "\n";
    std::cout << "P=\\left[";
    for (int j = 0; j < i; j++) {
        const auto& p = points.row(j);
        char buf[64];
        int n = sprintf(buf, "\\left(%.3f,\\ %.3f\\right)", p(0), p(1));
        if (i != j - 1) sprintf(buf + n, ", ");
        std::cout << buf;
        pros::delay(20);
    }
    std::cout << "\\right]" << "\n";

    float a = points.block(0, 0, i, 1).mean();
    float b = points.block(0, 1, i, 1).mean();
    printf("Center: (%.4f, %.4f)\n", -a, -b);
}

void controls::odom_centering::run() {
    int iteration = 0;

    while (true) {
        const bool cur = robot::master.get_digital(config::test::center_tick);
        if (cur && !last) {
            printf("[Odom Centering] Running iteration %d\n", ++iteration);
            tick();
            printf("[Odom Centering] Iteration %d complete\n", iteration);
            if (iteration == iterations) break;
        }
        last = cur;
        pros::delay(20);
    }
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