#include "hardware/motor.h"
#include "autonomous/controllers/velocity.h"
#include "device.h"
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

        if (group->braking) {
            pros::delay(10);
            continue;
        }

        // Ok this code is a little confusing so i will annotate it
        // Initially we assume target is in volts
        float target = group->target_value;
        float slew_range = group->slew_rate > 0 ? (group->slew_rate * dt) : infinityf();

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

        for (const auto& p : group->ports) {
            pros::c::motor_move_voltage(p, group->set_value);
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
    char name[64] = {0};
    sprintf(name, "motor_group_internal_manager%d", ports[0]);
    internal_management_task = pros::c::task_create(
        internal_management_func_motor_group, 
        this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, 
        name);
}

MotorGroup::MotorGroup (
    const std::vector<int>& ports, 
    Gearset gearset, 
    BrakeMode brake_mode,
    controllers::VelocityControllerArgs velo_controller_args,
    float slew_rate)
    : MotorGroup(ports, gearset, brake_mode, slew_rate)
{
    this->velo_controller.emplace(velo_controller_args);
}

MotorGroup::~MotorGroup() {
    AbstractDevice::~AbstractDevice();
    pros::c::task_delete(internal_management_task);
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
    } else {
        return velo_controller->reverse(target_value);
    }
    return 0;
}

void MotorGroup::brake(void) {
    if (!poll_mutex()) return;

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

void MotorGroup::set_brake_mode(pros::motor_brake_mode_e mode) {
    if (!acquire_mutex(0)) return;

    for (auto& motor : motors) {
        motor->set_brake_mode(mode);
    }
}

#define DEFINE_VECTOR_FUNCS(nm, ns, type) \
    std::vector<type> MotorGroup::nm(void) const { \
        std::vector<type> ret; \
        for (auto& motor : motors) ret.push_back(motor->ns()); \
        return ret; \
    }

DEFINE_VECTOR_FUNCS(get_raw_voltages, get_raw_voltage, float)
DEFINE_VECTOR_FUNCS(get_raw_velocities, get_raw_velocity, float)
DEFINE_VECTOR_FUNCS(get_filtered_velocities, get_filtered_velocity, float)
DEFINE_VECTOR_FUNCS(get_efficiencies, get_efficiency, float)
DEFINE_VECTOR_FUNCS(get_temperatures, get_temperature, float)
DEFINE_VECTOR_FUNCS(get_currents, get_current, float)
DEFINE_VECTOR_FUNCS(get_powers, get_power, float)
DEFINE_VECTOR_FUNCS(get_torques, get_torque, float)
DEFINE_VECTOR_FUNCS(get_positions, get_position, float)
DEFINE_VECTOR_FUNCS(get_brake_modes, get_brake_mode, BrakeMode)
DEFINE_VECTOR_FUNCS(get_ports, get_port, int)


