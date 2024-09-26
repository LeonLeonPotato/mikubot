#include "autonomous/strategy/test.h"
#include "autonomous/spline.h"
#include "autonomous/pathing.h"
#include "robot.h"

#include "api.h"

namespace test_strategy {
void run(void) {
    // spline::QuinticSpline sp;
    // sp.points.emplace_back(robot::x, robot::y);
    // sp.points.emplace_back(robot::x + 50, robot::y + 50);
    // sp.points.emplace_back(robot::x + 100, robot::y + 50);
    // sp.solve_coeffs(0, 0, 0, 0, 0, 0, 0, 0);

    // float fx = robot::x + 100;
    // float fy = robot::y + 50;
    // const float mult = (200 / M_PI);
    // float t = 0.1;

    // while (robot::distance(fx, fy) > 10) {
    //     Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    //     t = pure_pursuit::compute_intersections(
    //         sp, point, 25, t, 0, 2, 10
    //     );

    //     Eigen::Vector2f res = sp.compute(t);

    //     float dtheta = robot::angular_diff(res(0), res(1));
    //     float dist = robot::distance(res(0), res(1));

    //     float left = dist + mult * dtheta;
    //     float right = dist - mult * dtheta;

    //     robot::velo(left, right);

    //     pros::delay(20);
    // }

    // printf("Finished");
}
}