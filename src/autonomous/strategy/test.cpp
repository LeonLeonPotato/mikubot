#include "autonomous/strategy/test.h"
#include "autonomous/movement/simple/boomerang.h"
#include "autonomous/movement/simple/forward.h"
#include "autonomous/movement/simple/turn.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"

using namespace strategies;

static float START_OFFSET = 17.0f;

static void get_goal(void) {
    movement::simple::swing_to({0, -2*TILE + START_OFFSET}, {.reversed=true}, swing_group);
    swing_group.reset(); robot::brake();
}

static void long_path_part(void) {
    movement::simple::boomerang({-TILE * 2/3.0, TILE}, -M_PI, 0.5f, {}, swing_group);
    swing_group.reset(); robot::brake();

    // pathing::CubicSpline path;
    // path.points.emplace_back(-TILE * 2/3.0, TILE);
    // path.points.emplace_back(-TILE, 0);
    // path.points.emplace_back(-TILE-5, -TILE+10);
    // path.set_relative({0, -2*TILE + START_OFFSET});
    // path.points.insert(path.points.begin(), robot::pos);

    // path.solve_coeffs({{1, robot::theta, 200}}, path.natural_conditions);
    // path.profile_path(profile_params);
    // std::cout << path.debug_out() << std::endl;

    // movement::ramsete::RamseteParams params { 
    //     {
    //         .exit_threshold=5.0,
    //         .timeout=10000,
    //     }, 
    //     {
    //         .beta=2.0,
    //         .zeta=0.7
    //     } 
    // };

    // auto res = movement::ramsete::follow_path(path, params);
    // robot::brake();
}

static void go_back(void) {
    Eigen::Vector2f target = robot::pos + Eigen::Vector2f {0, TILE};
    movement::SimpleMovementParams params {
        .reversed = true, 
        .max_linear_speed = 0.8f
    };
    auto res = movement::simple::swing_to(target, params, swing_group);
    swing_group.reset(); robot::brake();
}

static void get_second_shit(void) {
    Eigen::Vector2f second = {-TILE + 5, -3*TILE + START_OFFSET + 10};
    movement::simple::turn_towards(
        second, 
        {
            .timeout=5000
        }, 
        in_place_pid
    );
    in_place_pid.reset(); robot::brake();

    movement::SimpleMovementParams params {
        .reversed = false, 
        .max_linear_speed = 0.8f
    };
    auto res = movement::simple::swing_to(second, params, swing_group);
    swing_group.reset(); robot::brake();
}

static void test_ramsete(void) {
    pathing::QuinticSpline path;
    path.points.emplace_back(0, 0);
    path.points.emplace_back(0, TILE);
    path.points.emplace_back(TILE, TILE);
    path.points.emplace_back(TILE, 0);
    path.set_relative(robot::pos);

    // path.solve_coeffs(pathing::NaturalCubicCondition, pathing::NaturalCubicCondition);
    path.profile_path(profile_params);

    movement::ramsete::RamseteParams params { 
        {
            .timeout=10000
        }, 
        {
            .beta=2.0,
            .zeta=0.7
        } 
    };

    auto res = movement::ramsete::follow_path(path, params);
    robot::brake();
}

void test_strategy::run(void) {
    //test_ramsete();
    // get_goal();
    // long_path_part();
    // go_back();
    // get_second_shit();
    // go_back();

    // movement::simple::turn_towards(M_PI/2, {.exit_threshold=rad(1.0), .timeout=2000}, in_place_pid);
    auto fut = movement::simple::boomerang_async({-50, 50}, 0, 0.5f, {.reversed = true, .linear_exit_threshold=2.0, .timeout=10000}, swing_group);

    std::vector<std::pair<Eigen::Vector2f, float>> poses;
    while (!fut.available()) {
        printf("pos: %f, %f | angle: %f\n", robot::pos.x(), robot::pos.y(), robot::theta);
        pros::delay(20);
        poses.push_back({robot::pos, robot::theta});
    }

    std::cout << fut.get().debug_out() << std::endl;

    std::cout << "P = \\left[";
    for (auto& p : poses)

    robot::brake();
}

