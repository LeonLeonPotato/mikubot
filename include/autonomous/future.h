#pragma once

#include "api.h"

template <typename T>
class Future {
    private:
        struct SharedState {
            std::vector<pros::task_t> waiting_tasks;
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
            for (auto& task : state->waiting_tasks) {
                pros::c::task_notify(task);
            }
        }

        void set_value(const T& value) {
            state->value = value;
            state->available = true;
            for (auto& task : state->waiting_tasks) {
                pros::c::task_notify(task);
            }
        }

        void wait() {
            if (!state->available && !state->cancelled) {
                state->waiting_tasks.push_back(pros::c::task_get_current());
                pros::c::task_notify_take(true, TIMEOUT_MAX);
            }
        }

        const T& get() {
            wait();
            return state->value;
        }

        bool valid() const {
            return !state->cancelled;
        }

        // use with caution
        std::shared_ptr<SharedState> get_state() {
            return state;
        }
};