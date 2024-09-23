#include "autonomous/strategy/test.h"
#include "autonomous/spline.h"
#include "robot.h"

#include "api.h"

namespace test_strategy {
void run() {
    spline::QuinticSpline sp;
    sp.points.emplace_back(robot::x, robot::y);
    sp.points.emplace_back(1, 1);
}
}