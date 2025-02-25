#pragma once

// namespace  {
// enum class State {
//     RESTING = 0,
//     PRIMED = 1,
//     FIRING = 2,
//     OVERRIDE = 3
// };

// void start_api_task();
// void stop_api_task();
// void exposed_go_to_state(State state);

// void run(void);
// void tick(void);
// void start_task(void);
// void pause(void);
// void resume(void);
// void stop_task(void);
// } // namespace controls::wallmech

#include "autonomous/controllers/pid.h"
#include "subsystems/base_system.h"

namespace subsystems {
class WallMech : public Subsystem {
    public:
        enum class State {
            RESTING = 0,
            PRIMED = 1,
            FIRING = 2,
            OVERRIDE = 3
        };

    private:
        static float positions[3];
        static WallMech* instance;

        int special_fire_thing = -1;
        State set_state = State::RESTING;
        controllers::PID pid {0.015f, 0.000f, 0.0004f};

    public:
        WallMech() {
            if (instance != nullptr) {
                throw std::runtime_error("Cannot create multiple instances of WallMech");
            }
            instance = this;
        }
        ~WallMech() {
            instance = nullptr;
        }

        void tick(void) override;
        void api_tick(void) override;
        bool has_api(void) const override { return true; }

        static WallMech& get_instance(void) { return *instance; }

        void exposed_go_to_state(State state);
};
}