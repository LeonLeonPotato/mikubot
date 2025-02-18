#pragma once

#include <math.h>
#include <string>
#include <cmath>
#include "pid.h"

namespace controllers {
struct VelocityControllerArgs {
    float kv, ka, kf;
    PIDArgs pid_args;
};

class VelocityController {
    private:
        PID pid;
        const VelocityControllerArgs args;
    
    public:        
        VelocityController(const VelocityControllerArgs& args) 
            : args(args), pid{args.pid_args} { reset(); }

        const VelocityControllerArgs& get_args(void) const {
            return args;
        }

        void reset(void) {
            pid.reset();
        }

        float get(float current, float target, float accel = 0) {
            float ff = args.kv * target + args.ka * accel + args.kf * (target > 0 ? 1 : -1) * (target != 0);
            float fb = pid.get(target - current);
            return ff + fb;
        }

        float get_no_update(float current, float target, float accel = 0) const {
            float ff = args.kv * target + args.ka * accel + args.kf * (target > 0 ? 1 : -1) * (target != 0);
            float fb = pid.get();
            return ff + fb;
        }

        float get(float target, float accel = 0) const {
            float ff = args.kv * target + args.ka * accel + args.kf * (target > 0 ? 1 : -1) * (target != 0);
            return ff;
        }

        // otherwise finds the steady state velocity for a voltage
        float reverse(float volt) const {
            if (fabsf(volt) < args.kf) return 0;
            return (volt - args.kf * (volt > 0 ? 1 : -1)) / args.kv;
        }
};
} // namespace controllers