#pragma once

#include "autonomous/controllers/velocity.h"
#include "device.h"
#include "mathtils.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include <cstdint>
#include <algorithm>
#include <optional>
#include <vector>

namespace hardware {
void internal_management_func_motor_group(void* args);

enum class Gearset {
    RED = 100,
    GREEN = 200,
    BLUE = 600
};

enum class BrakeMode {
    COAST = pros::E_MOTOR_BRAKE_COAST,
    BRAKE = pros::E_MOTOR_BRAKE_BRAKE,
    HOLD = pros::E_MOTOR_BRAKE_HOLD
};

enum class OutputUnit {
    VOLTAGE,
    RPM
};

class MotorGroup : public AbstractDevice {
    friend void internal_management_func_motor_group(void* args);

    private:
        Gearset gearset;
        BrakeMode brake_mode;
        float slew_rate;

        std::optional<controllers::VelocityController> velo_controller = std::nullopt;
        pros::task_t internal_management_task = nullptr;

        OutputUnit target_unit = OutputUnit::VOLTAGE;
        float target_value = 0;
        float target_accel_if_velocity_target = 0;
        OutputUnit set_unit = OutputUnit::VOLTAGE;
        float set_value = 0;

        bool braking = false;

    public:
        // 1/29/25: you better be using the same fuckin gearset on your drive
        // (dont be a ryan & or youll be sorry)
        MotorGroup(const std::vector<int>& ports, 
            Gearset gearset, 
            BrakeMode brake_mode,
            float slew_rate = 0);
        MotorGroup(const std::vector<int>& ports, 
            Gearset gearset, 
            BrakeMode brake_mode,
            const controllers::VelocityControllerArgs& velo_controller_args,
            float slew_rate = 0);

        ~MotorGroup();

        bool is_connected(void) const override;

        template <typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
        void set_desired_voltage(T voltage) {
            if (!poll_mutex()) return;

            target_unit = OutputUnit::VOLTAGE;
            target_value = std::clamp(static_cast<float>(voltage), -12000.0f, 12000.0f);
            target_accel_if_velocity_target = 0;
            braking = false;
        }
        float get_desired_voltage(void) const;
        float get_raw_voltage_average(void) const { return average(get_raw_voltages()); }
        std::vector<float> get_raw_voltages(void) const;

        template <typename T, typename U, 
            typename = std::enable_if_t<std::is_arithmetic<T>::value>,
            typename = std::enable_if_t<std::is_arithmetic<U>::value>>
        void set_desired_velocity(T rpm, U accel) {
            if (!poll_mutex()) return;

            target_unit = OutputUnit::RPM;
            target_value = std::clamp(static_cast<float>(rpm), 
                -static_cast<float>(gearset), 
                static_cast<float>(gearset));
            target_accel_if_velocity_target = static_cast<float>(accel);
            braking = false;
        }

        template <typename T, 
            typename = std::enable_if_t<std::is_arithmetic<T>::value>>
        void set_desired_velocity(T rpm) {
            if (!poll_mutex()) return;

            target_unit = OutputUnit::RPM;
            target_value = std::clamp(static_cast<float>(rpm), 
                -static_cast<float>(gearset), 
                static_cast<float>(gearset));
            target_accel_if_velocity_target = static_cast<float>(0);
            braking = false;
        }
        float get_desired_velocity(void) const;
        float get_raw_velocity_average(void) const { return average(get_raw_velocities()); }
        std::vector<float> get_raw_velocities(void) const;
        float get_filtered_velocity(void) const { return average(get_filtered_velocities()); }
        std::vector<float> get_filtered_velocities(void) const;

        float get_efficiency_average(void) const { return average(get_efficiencies()); }
        std::vector<float> get_efficiencies(void) const;
        float get_temperature_average(void) const { return average(get_temperatures()); }
        std::vector<float> get_temperatures(void) const;
        float get_current_average(void) const { return average(get_currents()); }
        std::vector<float> get_currents(void) const;
        float get_power_average(void) const { return average(get_powers()); }
        std::vector<float> get_powers(void) const;
        float get_torque_average(void) const { return average(get_torques()); }
        std::vector<float> get_torques(void) const;

        std::vector<float> get_positions(void) const;
        float get_position_average(void) const { return average(get_positions()); }
        void tare_position(void) const;
        void set_position(float position) const;

        void brake(void);
        void set_brake_mode(BrakeMode mode);
        void set_brake_mode(pros::motor_brake_mode_e mode) { set_brake_mode((BrakeMode) mode); }
        BrakeMode get_brake_mode(void) const { return brake_mode; }
        std::vector<BrakeMode> get_brake_modes_pros(void) const;

        Gearset get_gearset(void) const { return gearset; }
        float get_max_speed(void) const { return static_cast<float>(gearset); }

        void set_slew_rate(float rate) { slew_rate = rate; }
        float get_slew_rate(void) const { return slew_rate; }
};

class Motor : public MotorGroup {
    public:
        explicit Motor(int port, 
            Gearset gearset, 
            BrakeMode brake_mode,
            float slew_rate = 0)
            : MotorGroup({port}, gearset, brake_mode, slew_rate) {}
        explicit Motor(int port, 
            Gearset gearset, 
            BrakeMode brake_mode,
            controllers::VelocityControllerArgs velo_controller_args,
            float slew_rate = 0)
            : MotorGroup({port}, gearset, brake_mode, velo_controller_args, slew_rate) {}
};
} // namespace hardware