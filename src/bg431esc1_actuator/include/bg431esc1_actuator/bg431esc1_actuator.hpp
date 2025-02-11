#ifndef BG431ESC1_ACTUATOR_HPP
#define BG431ESC1_ACTUATOR_HPP

#include "bg431esc1_actuator_parameters.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include <cmath>
#include <cstdint>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <linux/can.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

#include "visibility_control.h"

namespace bg431esc1_actuator {
    class Bg431esc1Actuator : hardware_interface::ActuatorInterface {
    public:
        // LifecycleNodeInterface
        BG431ESC1_ACTUATOR_PUBLIC
        Bg431esc1Actuator(){};

        BG431ESC1_ACTUATOR_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        BG431ESC1_ACTUATOR_PUBLIC
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        BG431ESC1_ACTUATOR_PUBLIC
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

        BG431ESC1_ACTUATOR_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        BG431ESC1_ACTUATOR_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        BG431ESC1_ACTUATOR_PUBLIC
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        // ActuatorInterface
        BG431ESC1_ACTUATOR_PUBLIC
        CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

        BG431ESC1_ACTUATOR_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        BG431ESC1_ACTUATOR_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        BG431ESC1_ACTUATOR_PUBLIC
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> & start_interfaces,
            const std::vector<std::string> & stop_interfaces
        ) override;

        BG431ESC1_ACTUATOR_PUBLIC
        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string> & start_interfaces,
            const std::vector<std::string> & stop_interfaces
        ) override;

        BG431ESC1_ACTUATOR_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        BG431ESC1_ACTUATOR_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    private:
        const unsigned magic = 0b011000;

        enum class ControlMode : uint8_t {
            DISABLED,
            VOLTAGE,
            TORQUE,
            VELOCITY,
            POSITION,
            VELOCITY_OPEN_LOOP,
            POSITION_OPEN_LOOP
        };

        int m_socket;
        sockaddr_can m_address;

        struct Telemetry {
            double pos_estimate;
            double vel_estimate;
        };

        struct Status {
            int16_t applied_voltage;
            int16_t stator_current;
            int16_t supply_current;
            uint8_t bus_voltage;
            uint8_t driver_temp;
        };

        class BG431ESC1Data {
            unsigned m_can_address;
            ControlMode m_control_mode;

        public:
            Telemetry telemetry;
            Status status;
            std::string m_name;
            double m_setpoint;

            BG431ESC1Data(unsigned can_address, ControlMode control_mode, std::string name) {
                m_can_address = can_address;
                m_control_mode = control_mode;
                m_name = name;
            }
        };

        class BG431ESC1WithAuxData : public BG431ESC1Data {
        public:
            Telemetry aux_encoder;

            BG431ESC1WithAuxData(unsigned can_address, ControlMode control_mode, std::string name)
                : BG431ESC1Data(can_address, control_mode, name){};
        };

        std::vector<BG431ESC1Data> m_bg431esc1_data;
        std::vector<BG431ESC1WithAuxData> m_bg431esc1_with_aux_data;
    };
}

#endif // BG431ESC1_ACTUATOR_HPP