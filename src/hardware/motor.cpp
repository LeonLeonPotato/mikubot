#include "hardware/motor.h"
#include "autonomous/controllers/velocity.h"
#include "hardware/device.h"
#include "pros/apix.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <optional>
#include <sys/_intsup.h>
#include <sys/cdefs.h>
#include <vector>

using namespace hardware;

void hardware::internal_management_func_motor_group(void *args) {
    long long last_time = pros::micros();
    MotorGroup* group = static_cast<MotorGroup*>(args);

    while (true) {
        long long current_time = pros::micros();
        float dt = (current_time - last_time) / 1e6f;
        last_time = current_time;

        if (group->braking) {
            group->set_value = 0;
            pros::delay(10);
            continue;
        }

        // Ok this code is a little confusing so i will annotate it
        // Initially we assume target is in volts
        float target = group->target_value;
        float slew_range = group->slew_rate > 0 ? (group->slew_rate * dt) : 99999999.9f;

        if (group->target_unit == OutputUnit::RPM) { 
            if (group->velo_controller.has_value()) {
                /*
                If we have a custom velocity controller, we are controlling voltage
                So, since target is in RPM we will convert it to a voltage target outputted
                from the controller. This means our target and output will both be in volts
                from this point forwards.
                */
                target = group->velo_controller->get(
                    group->get_raw_velocity_average(), 
                    group->target_value, 
                    group->target_accel_if_velocity_target);
            } else {
                /*
                Here the user has not passed in a custom velocity controller, so we will use the
                built-in VEX controller onboard the motor.
                This means our output and target will both be in RPM / velocity from this point.
                */
                // If the last loop was in voltage output, we convert to velocity via reading from motors
                if (group->set_unit == OutputUnit::VOLTAGE) {
                    group->set_value = group->get_raw_velocity_average();
                }

                // Slew
                group->set_unit = OutputUnit::RPM;
                float max_speed = static_cast<float>(group->gearset);
                group->set_value = std::clamp(target, group->set_value - slew_range, group->set_value + slew_range);
                group->set_value = std::clamp(group->set_value, -max_speed, max_speed);

                for (const auto& p : group->ports) {
                    pros::c::motor_move_velocity(p, group->set_value);
                }

                // As below this point is handling for voltage-based output, we will do an early return
                pros::delay(10);
                continue;
            }
        }

        // Handle voltage-based outputting
        // If the last loop was in RPM output, we will convert it to voltage by reading from the motors
        if (group->set_unit == OutputUnit::RPM) {
            group->set_value = group->get_raw_voltage_average();
        }

        // Slew
        group->set_unit = OutputUnit::VOLTAGE;
        group->set_value = std::clamp(target, group->set_value - slew_range, group->set_value + slew_range);
        group->set_value = std::clamp(group->set_value, -12000.0f, 12000.0f);
        // printf("Set value:")

        for (const auto& p : group->ports) {
            pros::c::motor_move_voltage(p, group->set_value);
        }

        if (group->velo_controller.has_value()) {
            auto& args = group->velo_controller->get_args();
            float eff_volt = (group->set_value - args.kf * (group->set_value > 0 ? 1 : -1));
            if (eff_volt * group->set_value < 0) eff_volt = 0;

            float B = (1 / args.ka) * dt, A = 1 - (args.kv / args.ka) * dt; // (1 / (tc / gain)) * dt, 1 - (1 / tc) * dt
            float state_estimate_prior = group->kf.estimate_mean * A + eff_volt * B;
            float state_cov_prior = A * A * group->kf.estimate_cov + group->kf.process_cov;
            float reading = group->get_raw_velocity_average();
            float innovation = reading - group->kf.estimate_mean;
            float measurement_cov = group->kf.measurement_cov_factor * reading * reading + group->kf.measurement_cov_offset;
            float innovation_cov = group->kf.estimate_cov + measurement_cov * static_cast<float>(group->gearset) / 200.0f;
            float gain = state_cov_prior / (innovation_cov);
            group->kf.estimate_mean = state_estimate_prior + gain * innovation;
            group->kf.estimate_cov = (1 - gain) * state_cov_prior;
        }

        pros::delay(10);
    }
}

MotorGroup::MotorGroup (
    const std::vector<int>& ports, 
    Gearset gearset,
    BrakeMode brake_mode,
    float slew_rate)
    : AbstractDevice(ports), gearset(gearset), brake_mode(brake_mode), slew_rate(slew_rate)
{
    for (const auto& p : ports) {
        if (gearset == Gearset::BLUE) {
            pros::c::motor_set_gearing(p, pros::E_MOTOR_GEARSET_06);
        } else if (gearset == Gearset::GREEN) {
            pros::c::motor_set_gearing(p, pros::E_MOTOR_GEARSET_18);
        } else if (gearset == Gearset::RED) {
            pros::c::motor_set_gearing(p, pros::E_MOTOR_GEARSET_36);
        }
        pros::c::motor_set_encoder_units(p, pros::E_MOTOR_ENCODER_DEGREES);
    }

    for (const auto& p : ports) {
        pros::c::motor_set_brake_mode(p, (pros::motor_brake_mode_e_t) brake_mode);
        pros::c::motor_set_zero_position(p, pros::c::motor_get_position(p));
    }

    char name[64] = {0};
    sprintf(name, "motor_group_internal_manager%d", ports[0]);
    internal_management_task = pros::c::task_create(
        internal_management_func_motor_group, 
        this, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN, 
        name);
}

MotorGroup::MotorGroup (
    const std::vector<int>& ports, 
    Gearset gearset, 
    BrakeMode brake_mode,
    const controllers::VelocityControllerArgs& velo_controller_args,
    float slew_rate)
    : MotorGroup(ports, gearset, brake_mode, slew_rate)
{
    this->velo_controller.emplace(velo_controller_args);
}

MotorGroup::~MotorGroup() {
    AbstractDevice::~AbstractDevice();
    pros::c::task_delete(internal_management_task);
}

bool MotorGroup::is_connected(void) const {
    for (const auto& p : ports) {
        if (pros::c::get_plugged_type(p) != pros::c::E_DEVICE_MOTOR) 
            return false;
    }
    return true;
}

float MotorGroup::get_desired_voltage(void) const {
    if (target_unit == OutputUnit::VOLTAGE) {
        return target_value;
    } else if (velo_controller.has_value()) {
        return velo_controller->get_no_update(
            get_raw_velocity_average(), 
            target_value, 
            target_accel_if_velocity_target);
    }
    __unreachable();
}

float MotorGroup::get_desired_velocity(void) const {
    if (target_unit == OutputUnit::RPM) {
        return target_value;
    } else if (velo_controller.has_value() && target_unit == OutputUnit::VOLTAGE) {
        return velo_controller->reverse(target_value);
    } else {
        double sum = 0; double z = 0;
        for (const auto& p : ports) {
            double val = pros::c::motor_get_target_velocity(p);
            double y = val - z;
            double t = sum + y;
            z = (t - sum) - y;
            sum = t;
        }
        return static_cast<float>(sum / ports.size());
    }
    __unreachable();
}

void MotorGroup::set_position(float position) const {
    for (const auto& p : ports) {
        pros::c::motor_set_zero_position(p, pros::c::motor_get_position(p) - position);
    }
}

void MotorGroup::tare_position(void) const {
    for (const auto& p : ports) {
        pros::c::motor_set_zero_position(p, pros::c::motor_get_position(p));
    }
}

void MotorGroup::brake(void) {
    if (!poll_mutex()) return;

    braking = true;
    for (const auto& p : ports) {
        pros::c::motor_brake(p);
    }
}

void MotorGroup::set_brake_mode(BrakeMode mode) {
    if (!poll_mutex()) return;

    for (const auto& p : ports) {
        pros::c::motor_set_brake_mode(p, (pros::motor_brake_mode_e_t) mode);
    }
}

#define DEFINE_VECTOR_FUNCS(nm, ns, type) \
    std::vector<type> MotorGroup::nm(void) const { \
        std::vector<type> ret; \
        for (const auto& p : ports) ret.push_back(static_cast<type>(pros::c::ns(p))); \
        return ret; \
    }

DEFINE_VECTOR_FUNCS(get_raw_voltages, motor_get_voltage, float)
DEFINE_VECTOR_FUNCS(get_raw_velocities, motor_get_actual_velocity, float)
DEFINE_VECTOR_FUNCS(get_efficiencies, motor_get_efficiency, float)
DEFINE_VECTOR_FUNCS(get_temperatures, motor_get_temperature, float)
DEFINE_VECTOR_FUNCS(get_currents, motor_get_current_draw, float)
DEFINE_VECTOR_FUNCS(get_powers, motor_get_power, float)
DEFINE_VECTOR_FUNCS(get_torques, motor_get_torque, float)
DEFINE_VECTOR_FUNCS(get_positions, motor_get_position, float)
DEFINE_VECTOR_FUNCS(get_brake_modes_pros, motor_get_brake_mode, BrakeMode)


