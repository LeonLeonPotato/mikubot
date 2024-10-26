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

        void set_value(const T& value) {
            state->value = value;
            state->available = true;
        }

        void wait() {
            while (!state->available) {
                pros::delay(5);
            }
        }

        const T& get() {
            wait();
            return state->value;
        }

        bool valid() const {
            return !state->cancelled;
        }

        bool available() const {
            return state->available;
        }

        // use with caution
        std::shared_ptr<SharedState> get_state() {
            return state;
        }
};