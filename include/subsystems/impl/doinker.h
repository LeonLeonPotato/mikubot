#pragma once

#include "subsystems/base_system.h" // IWYU pragma: export

namespace subsystems {
class Doinker : public Subsystem {
    private:
        static Doinker* instance;

    public:
        Doinker() {
            if (instance != nullptr) {
                throw std::runtime_error("Cannot create multiple instances of Doinker");
            }
            instance = this;
        }
        ~Doinker() {
            instance = nullptr;
        }

        void tick(void) override;
        bool has_api(void) const override { return false; }

        static Doinker& get_instance(void) { return *instance; }
};
} // namespace controls::doinker