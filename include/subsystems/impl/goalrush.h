#pragma once

#include "subsystems/base_system.h" // IWYU pragma: export

namespace subsystems {
class Goalrush : public Subsystem {
    private:
        static Goalrush* instance;

    public:
        Goalrush() {
            if (instance != nullptr) {
                throw std::runtime_error("Cannot create multiple instances of Goalrush");
            }
            instance = this;
        }
        ~Goalrush() {
            instance = nullptr;
        }

        void tick(void) override;
        bool has_api(void) const override { return false; }

        static Goalrush& get_instance(void) { return *instance; }
};
} // namespace controls::doinker