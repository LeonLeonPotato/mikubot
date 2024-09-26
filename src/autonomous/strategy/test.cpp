#include "autonomous/strategy/test.h"
#include "autonomous/spline.h"
#include "autonomous/pathing.h"
#include "robot.h"

#include "api.h"

namespace test_strategy {
spline::QuinticSpline sp;

float recompute_full() {
    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::VectorXf guesses = Eigen::VectorXf(20);
    guesses.setLinSpaced(20, 0.1, 1.9);
    return pure_pursuit::compute_intersections(
        sp, point, 50, guesses, 0, sp.points.size()-1, 15
    );
}

void run(void) {
    int it = 0;
    sp = spline::QuinticSpline();
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(robot::x, robot::y + 200);
    sp.points.emplace_back(robot::x + 200, robot::y + 200);
    sp.solve_coeffs(0, 0, 0, 0, 0, 0, 0, 0);

    const float mult = (4*127 / M_PI);
    float t = recompute_full();

    while (true) {
        float fdist = robot::distance(sp.points.back()[0], sp.points.back()[1]);
        if (fdist < 10) {
            break;
        }

        Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
        t = pure_pursuit::compute_intersections(
            sp, point, 50, t, 0, sp.points.size()-1, 15
        );
        if (t == -1.0) {
            sp.points[0] = point;
            sp.solve_coeffs(0, 0, 0, 0, 0, 0, 0, 0);
            t = recompute_full();
        }

        Eigen::Vector2f res = sp.compute(t);

        float dtheta = robot::angular_diff(res(0), res(1));
        float dist = robot::distance(res(0), res(1));
        dist = fmin(dist * dist, 127);

        int left = (int) (dist - mult * dtheta);
        int right = (int) (dist + mult * dtheta);

        if (it % 10 == 0) {
            printf("t = %f, Robot = [%f, %f], fdist = %f, res = [%f, %f], dtheta = %f, dist = %f, velo = [%d, %d]\n", 
                    t, point(0), point(1), fdist, res(0), res(1), dtheta, dist, left, right);
        }

        robot::velo(left, right);

        // printf("Robot at [%f, %f], fdist = %f, res = [%f, %f], dtheta = %f, dist = %f, velo = [%f, %f]\n", robot::x, robot::y, robot::distance(fx, fy), res(0), res(1), dtheta, dist, left, right);

        pros::delay(20);
        it ++;
    }

    printf("Finished\n");
}
}