#pragma once

#include "Eigen/Dense"
#include "pros/rtos.h"
#include "pose.h"

namespace odometry {
class BaseOdometry {
    private:
        static void instance_caller_func(void* args) {
            static_cast<BaseOdometry*>(args)->run_task();
        }

        pros::task_t task = nullptr;

    protected:
        bool tracking_pose_updated = false;
        pros::mutex_t tracking_pose_mutex;
        Pose tracking_pose;

        BaseOdometry(void) : tracking_pose(0, 0, 0) {}
        BaseOdometry(const Pose& start_pose) : tracking_pose(start_pose) {}

        virtual void run_task(void) = 0;

    public:
        void start_task(void) {
            if (task != nullptr) return;
            // task = pros::c::task_create(instance_caller_func, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
        }
        
        void stop_task(void) {
            if (task == nullptr) return;
            pros::c::task_delete(task);
            task = nullptr;
        }
        
        bool is_running(void) const {
            return task != nullptr;
        }
        
        void set_pose(float x, float y, float theta) {
            pros::c::mutex_take(tracking_pose_mutex, TIMEOUT_MAX);
            tracking_pose.set_pos(Eigen::Vector2f(x, y));
            tracking_pose.set_theta(theta);
            tracking_pose_updated = true;
            pros::c::mutex_give(tracking_pose_mutex);
        }

        void set_pose(const Eigen::Vector2f& pos, float theta) {
            pros::c::mutex_take(tracking_pose_mutex, TIMEOUT_MAX);
            tracking_pose.set_pos(pos);
            tracking_pose.set_theta(theta);
            tracking_pose_updated = true;
            pros::c::mutex_give(tracking_pose_mutex);
        }

        void set_pose(const Pose& pose) {
            pros::c::mutex_take(tracking_pose_mutex, TIMEOUT_MAX);
            tracking_pose = pose;
            tracking_pose_updated = true;
            pros::c::mutex_give(tracking_pose_mutex);
        }

        Pose get_pose(void) const {
            return tracking_pose;
        }
};
}