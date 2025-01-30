#pragma once

#include "subsystems/base_system.h"

namespace subsystems {
class Clamp : public Subsystem {
    private:
        static Clamp* instance;

    public:
        Clamp() {
            if (instance != nullptr) {
                throw std::runtime_error("Cannot create multiple instances of Clamp");
            }
            instance = this;
        }
        ~Clamp() {
            instance = nullptr;
        }

        void tick(void) override;
        bool has_api(void) const override { return false; }
};
} // namespace subsystems::clamp