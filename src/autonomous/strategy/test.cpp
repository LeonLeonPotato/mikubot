#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

namespace test_strategy {
pathing::QuinticSpline sp;
const float mult = 150;
const float radius = 30;
const float recalculate_velo_mult = 0;

float recompute_full() {
    // DO NOT CHANGE SIN AND COS
    // This is because (0, 1) is defined as theta = 0, which means we MUST have sin = x and cos = y
    float dir_x = sinf(robot::theta) * recalculate_velo_mult;
    float dir_y = cosf(robot::theta) * recalculate_velo_mult;
    sp.solve_coeffs(dir_x, 0, dir_y, 0, 0, 0, 0, 0);

    Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
    Eigen::VectorXf guesses = Eigen::VectorXf(30);
    guesses.setLinSpaced(30, 0.05, sp.points.size()-1.05);
    return movement::pure_pursuit::compute_intersections(
        sp, point, radius, guesses, 0, sp.points.size()-1, 15
    ).first;
}

void run(void) {
    long long it = 0;
    sp = pathing::QuinticSpline();
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(robot::x, robot::y + 100);
    sp.points.emplace_back(robot::x + 50, robot::y + 200);
    sp.points.emplace_back(robot::x + 200, robot::y + 200);

    std::vector<Eigen::Vector2f> tracker_res;
    std::vector<Eigen::Vector2f> tracker_pos;
    controllers::PID pid; movement::init_pid(pid);

    float t = recompute_full();
    long long start = pros::micros();

    while (pros::micros() - start < 10 * 1e6) {
        float fdist = robot::distance(sp.points.back());
        if (fdist < 10) {
            break;
        }

        Eigen::Vector2f point = Eigen::Vector2f(robot::x, robot::y);
        auto intersect = movement::pure_pursuit::compute_intersections(
            sp, point, radius, t, t, sp.points.size()-1
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

        movement::goto_pos_tick(res, pid);
        tracker_pos.push_back(point);
        tracker_res.push_back(res);
        printf("Robot pos: (%f, %f) | Res: (%f, %f)\n", robot::x, robot::y, res(0), res(1));

        pros::delay(20);
        it++;
    }
    robot::brake();

    printf("Finished\n");

    printf("P = [");
    for (int i = 0; i < tracker_pos.size(); i++) {
        printf("(%f, %f), ", tracker_pos[i](0), tracker_pos[i](1));
    }
    printf("]\n");
    printf("R = [");
    for (int i = 0; i < tracker_res.size(); i++) {
        printf("(%f, %f), ", tracker_res[i](0), tracker_res[i](1));
    }
    printf("]\n");
}
}