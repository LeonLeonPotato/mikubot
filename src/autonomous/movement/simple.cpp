#include "autonomous/movement/simple.h"
#include "essential.h"

using namespace movement;

TickResult simple::turn_towards_tick(float angle, controllers::PID& pid,
    int timeout, float threshold)
{
    float diff = robot::angular_diff(angle);
    int ctrl = (int) pid.get(diff);
    robot::volt(ctrl, -ctrl);
    return { ExitCode::SUCCESS, 0, diff, false };
}

MovementResult simple::turn_towards_cancellable(float angle, controllers::PID& pid,
    bool& cancel_ref, int timeout, float threshold)
{
    MovementResult result;

    int start = pros::millis();
    while (true) {
        if (cancel_ref) {
            result.code = ExitCode::CANCELLED;
            break;
        }

        float diff = robot::angular_diff(angle);
        if (fabs(diff) < threshold) {
            result.code = ExitCode::SUCCESS;
            break;
        }

        if (pros::millis() - start > timeout) {
            result.code = ExitCode::TIMEOUT;
            break;
        }

        turn_towards_tick(angle, pid, timeout, threshold);
    }

    result.time_taken_ms = pros::millis() - start;

    return result;
}