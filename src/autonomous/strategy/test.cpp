#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

movement::PurePursuit pure_pursuit = movement::PurePursuit(
    
);

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

    auto pp = movement::PurePursuit(qs, 90);
    pp.params.always_recompute = true;
    // printf("%f", ((movement::PurePursuitParams&) pp.params).radius);
    auto fut = pp.follow_path_async();

    // std::cout << pure_pursuit.params.timeout << std::endl;

    int start = pros::millis();
    while (!fut.available()) {
        pros::delay(20);
    }

    printf("%s", fut.get().debug_out().c_str());
    std::cout << qs.debug_out() << std::endl;
}