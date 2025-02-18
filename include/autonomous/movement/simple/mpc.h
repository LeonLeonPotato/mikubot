#pragma once

#include "autonomous/movement/base_movement.h"
#include "hardware/motor.h"
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
    float max_time = 20;
    float dt;
};

struct DiffdriveMPCParams : public MPCParams {
    float track_width, gain, tc;
};

struct DiffdrivePenalty {
    float x, y, theta, vl = 0, vr = 0;
};

template <typename T>
struct DiffdriveState {
    T x, y, theta, vl, vr;

    DiffdriveState<T> operator-(const DiffdriveState<T>& other) const {
        return {x - other.x, y - other.y, theta - other.theta, vl - other.vl, vr - other.vr};
    }

    DiffdriveState<T> operator+(const DiffdriveState<T>& other) const {
        return {x + other.x, y + other.y, theta + other.theta, vl + other.vl, vr + other.vr};
    }

    T sum(void) const {
        return x + y + theta + vl + vr;
    }

    DiffdriveState<T> operator*(const T& scalar) const {
        return {x * scalar, y * scalar, theta * scalar, vl * scalar, vr * scalar};
    }

    DiffdriveState<T> operator*(const DiffdriveState<T>& other) const {
        return {x * other.x, y * other.y, theta * other.theta, vl * other.vl, vr * other.vr};
    }

    DiffdriveState<T> operator*(const DiffdrivePenalty& penality) const {
        return {x * penality.x, y * penality.y, theta * penality.theta, vl * penality.vl, vr * penality.vr};
    }
};

template <int N>
void test_motor(
    hardware::Motor& motor,
    const DiffdrivePenalty& penalty
);
}