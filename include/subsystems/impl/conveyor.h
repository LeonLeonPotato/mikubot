#pragma once

#include "subsystems/base_system.h"

namespace subsystems {
class Conveyor : public Subsystem {
    private:
        static Conveyor* instance;

        int disable_length = 200;
        bool color_sort_enabled = true;
        int desired_voltage = 0;

    public:
        Conveyor() {
            if (instance != nullptr) {
                throw std::runtime_error("Cannot create multiple instances of Conveyor");
            }
            instance = this;
        }
        ~Conveyor() {
            instance = nullptr;
        }

        void tick(void) override;
        void api_tick(void) override;
        bool has_api(void) const override { return true; }

        void set_disabled_length(int length) {
            // if (!poll_mutex()) return;
            disable_length = length;
        }
        int get_disabled_length(void) const { return disable_length; }
        void set_color_sort_enabled(bool enabled) {
            // if (!poll_mutex()) return;
            color_sort_enabled = enabled;
        }
        bool get_color_sort_enabled(void) const { return color_sort_enabled; }
        void toggle_color_sort_enabled(void) {
            // if (!poll_mutex()) return;
            color_sort_enabled = !color_sort_enabled;
        }

        void set_desired_voltage(int voltage) {
            // if (!poll_mutex()) return;
            desired_voltage = voltage;
        }
        int get_desired_voltage(void) const { return desired_voltage; }

        static Conveyor& get_instance(void) { return *instance; }
};
} // namespace controls::conveyor