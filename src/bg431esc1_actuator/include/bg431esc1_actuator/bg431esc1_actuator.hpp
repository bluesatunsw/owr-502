#ifndef BG431ESC1_ACTUATOR_HPP
#define BG431ESC1_ACTUATOR_HPP

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
        static constexpr unsigned magic_number = 0b011000;

        bool enabled = false;

        int m_sockfd;
        sockaddr_can m_socket;        

        struct CanId {
            CanId(uint8_t address, uint8_t api_page, uint8_t api_index) {
                this->address = address;
                this->api_page = api_page;
                this->api_index = api_index;
            }

            CanId(uint8_t address, uint8_t api_page, uint8_t api_index, uint8_t anonymous) {
                this->address = address;
                this->api_page = api_page;
                this->api_index = api_index;
                this->anonymous = anonymous;
            }

            CanId(uint8_t address, uint8_t api_page, uint8_t api_index, uint8_t anonymous, uint8_t priority) {
                this->address = address;
                this->api_page = api_page;
                this->api_index = api_index;
                this->anonymous = anonymous;
                this->priority = priority;
            }

            uint8_t priority: 3 = 4;
            uint8_t anonymous: 1 = 0b0;
            uint8_t magic: 6 = magic_number;
            uint8_t api_page: 2;
            uint16_t api_index: 10;
            uint8_t reserved: 1 = 0b1;
            uint8_t address: 6;
        };

        enum class ControlMode : uint8_t {
            DISABLED,
            VOLTAGE,
            TORQUE,
            VELOCITY,
            POSITION,
            VELOCITY_OPEN_LOOP,
            POSITION_OPEN_LOOP
        };

        struct Control {
            ControlMode _control_mode: 8;
            uint32_t _setpoint: 32;
        };

        struct Status0 {
            uint32_t _position: 32;
            uint32_t _velocity: 32;
        };

        struct Status2 {
            int16_t applied_voltage: 16;
            int16_t stator_current: 16;
            int16_t supply_current: 16;
            uint8_t bus_voltage: 8;
            uint8_t driver_temp;
        };

        class BG431ESC1Data {
            ControlMode m_control_mode;
            double m_sensor_scale = 1.0;

        public:
            Status0 m_telemetry;
            Status0 aux_telemetry;
            Status2 m_status;
            std::string m_name;
            uint8_t m_can_address;

            bool m_has_aux = false;
            double m_setpoint;
            double m_position_estimate;
            double m_velocity_estimate;

            double m_aux_position_estimate;
            double m_aux_velocity_estimate;

            BG431ESC1Data(uint8_t can_address, ControlMode control_mode, std::string name, bool has_aux) {
                m_can_address = can_address;
                m_control_mode = control_mode;
                m_name = name;
                m_has_aux = has_aux;
            }

            void set_sensor_scale(double scale) {
                m_sensor_scale = scale;
            }

            bool send_frame() {
                CanId id = CanId(m_can_address, 0b01, 0b0000000001);
            }

        };

        std::vector<BG431ESC1Data> m_bg431esc1_data;
    };
}

#endif // BG431ESC1_ACTUATOR_HPP