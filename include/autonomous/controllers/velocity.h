#pragma once

#include <math.h>
#include <string>
#include <cmath>
#include "pid.h"

namespace controllers {
struct VelocityControllerArgs {
    float kv, ka, kf;
};

class VelocityController {
    public:
        PID pid;
        VelocityControllerArgs args;
        
        VelocityController(float kv, float ka, float kf,
                            float kp, float ki, float kd) : args{kv, ka, kf}, pid{kp, ki, kd} { reset(); }
        VelocityController(float kv, float ka, float kf) : args{kv, ka, kf}, pid({}) { reset(); }
        VelocityController(const VelocityControllerArgs& args, const PIDArgs& pid_args) : args(args), pid{pid_args} { reset(); }
        VelocityController(const VelocityControllerArgs& args) : args(args), pid({}) { reset(); };
        VelocityController(const VelocityControllerArgs& args, PID pid) : args(args), pid(pid) { reset(); };

        void reset(void) {
            pid.reset();
        }

        float get(float current, float target, float accel = 0) {
            float ff = args.kv * target + args.ka * accel + args.kf * (target > 0 ? 1 : -1) * (target != 0);
            float fb = pid.get(target - current);
            return ff + fb;
        }

        float get(float target, float accel = 0) const {
            float ff = args.kv * target + args.ka * accel + args.kf * (target > 0 ? 1 : -1) * (target != 0);
            return ff;
        }

        float reverse(float volt) const {
            if (fabsf(volt) < args.kf) return 0;
            return (volt - args.kf * (volt > 0 ? 1 : -1)) / args.kv;
        }
};
} // namespace controllers