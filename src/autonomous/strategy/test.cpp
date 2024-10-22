#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

namespace test_strategy {
pathing::QuinticSpline sp;

void run(void) {
    sp = pathing::QuinticSpline();
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(robot::x, robot::y + 100);
    sp.points.emplace_back(robot::x + 50, robot::y + 200);
    sp.points.emplace_back(robot::x + 200, robot::y + 200);

    movement::pure_pursuit::follow_path(sp, 30);
}
}