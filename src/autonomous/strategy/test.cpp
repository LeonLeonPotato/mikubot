#include "autonomous/strategy/test.h"
#include "autonomous/spline.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

namespace test_strategy {
spline::QuinticSpline sp;
const float mult = 160;
const float radius = 50;
const float recalculate_velo_mult = 10;

float recompute_full() {
    // DO NOT CHANGE SIN AND COS
    // This is because (0, 1) is defined as theta = 0, which means we MUST have sin = x and cos = y
    float dir_x = sinf(robot::theta) * recalculate_velo_mult;
    float dir_y = cosf(robot::theta) * recalculate_velo_mult;
    sp.solve_coeffs(dir_x, 0, dir_y, 0, 0, 0, 0, 0);

    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::VectorXf guesses = Eigen::VectorXf(20);
    guesses.setLinSpaced(20, 0.1, 1.9);
    return pure_pursuit::compute_intersections(
        sp, point, 50, guesses, 0, sp.points.size()-1, 15
    );
}

void run(void) {
    freopen("output.txt", "w", stdout);
    long long it = 0;
    sp = spline::QuinticSpline();
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(robot::x, robot::y + 200);
    sp.points.emplace_back(robot::x + 200, robot::y + 200);

    float t = recompute_full();

    while (true) {
        float fdist = robot::distance(sp.points.back());
        if (fdist < 10) {
            break;
        }

        Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
        t = pure_pursuit::compute_intersections(
            sp, point, 50, t, 0, sp.points.size()-1, 5
        );
        if (t == -1.0) {
            sp.points[0] = point;
            t = recompute_full();
        }

        Eigen::Vector2f res = sp.compute(t);

        float dtheta = robot::angular_diff(res);
        float dist = robot::distance(res);
        dist = fmin(dist * dist, 127);

        int left = (int) (dist + mult * dtheta);
        int right = (int) (dist - mult * dtheta);

        if (it % 10 == 0) {
            printf("t = %f, Robot = [%f, %f], fdist = %f, res = [%f, %f], dtheta = %f, dist = %f, velo = [%d, %d]\n", 
                    t, point(0), point(1), fdist, res(0), res(1), dtheta, dist, left, right);
        }

        // robot::velo(left, right);

        pros::delay(20);
        it++;
    }

    printf("Finished\n");
}
}