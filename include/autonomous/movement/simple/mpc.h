#pragma once

#include "autonomous/movement/base_movement.h"
#include "pose.h"
#include "simpletils.h"
#include "nlopt/nlopt.hpp"

namespace movement::simple {
enum class MPCScaling {
    NONE,
    LINEAR,
    QUADRATIC,
    EXPONENTIAL
};

struct MPCParams {
    nlopt::algorithm alg;
    MPCScaling scaling = MPCScaling::LINEAR;
    float ftol_rel = 0.01;
    float max_time = 0.02;
    float dt;
};

struct DiffdriveMPCParams : public MPCParams {
    float track_width, gain, tc;
};

template <typename T>
struct DiffdriveState {
    T x, y, theta, vl, vr;
};

struct Penalty {
    float x, y, theta, vl = 0, vr = 0;
};

struct TestMotorMPCParams : public MPCParams {
    float gain, tc;
};
}