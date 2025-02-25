#include "autonomous/strategy/mpcstrat.h"
#include "autonomous/movement/simple/mpc.h"
#include "autonomous/strategy/utils.h"
#include "essential.h"

using namespace strategies;

void mpcstrat::run(void) {
    robot::chassis.take_drive_mutex();

    const auto mpc_params = movement::simple::DiffdriveMPCParams         {
        .track_width = 30.29204,
        .gain = 57.2419807724,
        .tc = 0.137,
        .kf = 0.700,
        .linear_mult = 4.1275 * 0.75,
    };

    movement::simple::follow_recording<4>(
        "log1.txt", 
        mpc_params,
        {
            .x = 1.0,
            .y = 1.0,
            .theta = 20.0,
            .vl = 0.000001,
            .vr = 0.000001,
            .u_penalty = 0.000000002,
            .u_diff_penalty = 0.000
        }
    );

    robot::chassis.give_drive_mutex();
}