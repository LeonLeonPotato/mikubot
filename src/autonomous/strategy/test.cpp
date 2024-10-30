#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

void test_strategy::run(void) {
    robot::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    auto qs = pathing::QuinticSpline();
    qs.points.emplace_back(robot::x, robot::y);
    qs.points.emplace_back(robot::x - 53, robot::y + 136);
    qs.points.emplace_back(robot::x - 15, robot::y + 269);
    qs.points.emplace_back(robot::x + 100.6, robot::y + 308.5);
    qs.points.emplace_back(robot::x + 191, robot::y + 263);
    qs.points.emplace_back(robot::x + 195, robot::y + 106);
    qs.points.emplace_back(robot::x + 96, robot::y + 35.5);

    auto pp = movement::PurePursuit(&qs, 10);
    std::cout << reinterpret_cast<uintptr_t>(&qs) << std::endl;
    pp.path = &qs;
    std::cout << reinterpret_cast<uintptr_t>(pp.path) << std::endl;
    auto fut = pp.follow_path();
    if (1==1) return;

    // std::cout << pure_pursuit.params.timeout << std::endl;

    // int start = pros::millis();
    // while (!fut.available()) {
    //     // printf("asdasdsadasd %d\n", pros::millis());
    //     pros::delay(20);
    // }

    // std::cout << qs.debug_out() << std::endl;

    // auto val = fut.get();
    // printf("QS Pure pursuit Result: %s\n", val.debug_out().c_str());

    // auto gtres = movement::simple::turn_towards(-M_PI / 2);
    // printf("Go to Result: %s\n", gtres.debug_out().c_str());
    // robot::brake();

    // auto pathback = pathing::BoomerangPath(
    //     Eigen::Vector2f(robot::x, robot::y), Eigen::Vector2f(0, 0)
    // );

    // auto pb_initializer = [](pathing::BaseParams& params) {
    //     params.start_heading = 0;
    //     params.start_magnitude = 0;
    //     params.end_heading = -M_PI;
    //     params.end_magnitude = 1 / sqrtf(2);
    // };
    // auto res = movement::PurePursuit(&pathback, 10, pb_initializer).follow_path();
    // printf("Pathing back Result: %s\n", res.debug_out().c_str());

    // robot::brake();
    // robot::set_brake_mode(robot::config::default_brake_mode);

    // printf("Finished test strategy\n");
}