#pragma once

#include "subsystems/base_system.h"

namespace subsystems {
class Driving : public Subsystem {
    private:
        static Driving* instance;

    public:
        Driving() {
            if (instance != nullptr) {
                throw std::runtime_error("Cannot create multiple instances of Driving");
            }
            instance = this;
        }
        ~Driving() {
            instance = nullptr;
        }

        void tick(void) override;
        bool has_api(void) const override { return false; }

        static Driving& get_instance(void) { return *instance; }
};
} // namespace subsystems::driving