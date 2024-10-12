#include "autonomous/strategy/test.h"
#include "autonomous/spline.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

namespace test_strategy {
spline::QuinticSpline sp;
const float mult = 400;
const float radius = 30;
const float recalculate_velo_mult = 0;

float recompute_full() {
    // DO NOT CHANGE SIN AND COS
    // This is because (0, 1) is defined as theta = 0, which means we MUST have sin = x and cos = y
    float dir_x = sinf(robot::theta) * recalculate_velo_mult;
    float dir_y = cosf(robot::theta) * recalculate_velo_mult;
    sp.solve_coeffs(dir_x, 0, dir_y, 0, 0, 0, 0, 0);

    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::VectorXf guesses = Eigen::VectorXf(20);
    guesses.setLinSpaced(20, 0.1, sp.points.size()-1.1);
    return pure_pursuit::compute_intersections(
        sp, point, radius, guesses, 0, sp.points.size()-1, 15
    );
}

void run(void) {
    long long it = 0;
    sp = spline::QuinticSpline();
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(robot::x, robot::y + 100);
    sp.points.emplace_back(robot::x - 50, robot::y + 100);
    Eigen::Vector2f back = sp.points.back();

    std::vector<Eigen::Vector2f> tracker_res;
    std::vector<Eigen::Vector2f> tracker_pos;

    float t = recompute_full();
    std::cout << sp.debug_out() << std::endl;

    while (true) {
        float fdist = robot::distance(back);
        if (fdist < 10) {
            break;
        }

        Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
        auto intersect = pure_pursuit::compute_intersections(
            sp, point, radius, t, t, sp.points.size()-1, 5
        );
        t = intersect.first;
        if (intersect.second > radius + 5) {
            sp.points[0] = point;
            for (auto& p : sp.points) {
                std::cout << p(0) << " " << p(1) << std::endl;
            }
            t = recompute_full();
            std::cout << "Recomputed " << t << std::endl;
        }

        Eigen::Vector2f res = sp.compute(t);

        float dtheta = robot::angular_diff(res);
        float dist = robot::distance(res);
        dist = fmin(dist * 5, radius) / radius * 50;

        int left = (int) (dist + mult * dtheta);
        int right = (int) (dist - mult * dtheta);

        if (it % 5 == 0) {
            printf("t = %f, Robot = [%f, %f], fdist = %f, res = [%f, %f], dtheta = %f, dist = %f, velo = [%d, %d]\n", 
                    t, point(0), point(1), fdist, res(0), res(1), dtheta, dist, left, right);

            tracker_res.push_back(res);
            tracker_pos.push_back(point);

            std::cout << "[";
            for (int i = 0; i < tracker_res.size(); i++) {
                std::cout << "(" << tracker_res[i](0) << ", " << tracker_res[i](1) << "), ";
            }
            std::cout << "]\n";
            std::cout << "[";
            for (int i = 0; i < tracker_res.size(); i++) {
                std::cout << "(" << tracker_pos[i](0) << ", " << tracker_pos[i](1) << "), ";
            }
            std::cout << "]\n";
        }

        robot::volt(left, right);

        pros::delay(20);
        it++;
    }
    robot::brake();

    printf("Finished\n");
}
}