#include "autonomous/strategy/test.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"

#include "api.h"

using namespace strategies;

pathing::QuinticSpline sp;

void test_strategy::run(void) {
    sp = pathing::QuinticSpline();
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(robot::x, robot::y + 100);
    sp.points.emplace_back(robot::x + 50, robot::y + 200);
    sp.points.emplace_back(robot::x + 200, robot::y + 200);

    movement::stanley::follow_path(sp);
}