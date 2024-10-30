#pragma once

#include "api.h"

template <typename T>
class Future {
    private:
        struct SharedState {
            bool cancelled = false;
            bool available = false;
            T value;
        };

        std::shared_ptr<SharedState> state;

    public:
        Future() {
            state = std::make_shared<SharedState>();
        }

        void cancel() {
            state->cancelled = true;
        }

        void set_value(T&& value) {
            state->value = std::move(value);
            state->available = true;
        }

        void set_value(const T& value) {
            state->value = value;
            state->available = true;
        }

        bool wait(int timeout = TIMEOUT_MAX, int delay = 5) const {
            int start = pros::millis();
            while (!state->available && pros::millis() - start < timeout) {
                pros::delay(delay);
            }
            return state->available;
        }

        T& get(int timeout = TIMEOUT_MAX, int delay = 5) const {
            wait(timeout, delay);
            return state->value;
        }

        bool valid() const {
            return !state->cancelled;
        }

        bool available() const {
            return state->available;
        }

        // use with caution
        std::shared_ptr<SharedState> get_state() const {
            return state;
        }
};