#include "autonomous/strategy/peak_strat.h"
#include "autonomous/movement.h"
#include "autonomous/pathing.h"
#include "essential.h"
#include "config.h"

#include "api.h"

using namespace strategies;

static const controllers::PIDArgs linear_args {
    .kp = 0.8,
    .ki = 0,
    .kd = -0.01
};

static const controllers::PIDArgs angular_args {
    .kp = 0.4,
    .ki = 0,
    .kd = 0
};

static const controllers::PIDArgs in_place_args {
    .kp = 1,
    .ki = 0,
    .kd = -0.025
};

void peak_strat::run(void) {
    
}

