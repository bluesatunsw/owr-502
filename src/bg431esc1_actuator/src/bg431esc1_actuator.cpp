#include "bg431esc1_actuator/bg431esc1_actuator.hpp"

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>
#include <linux/can.h>
#include <memory>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>
#include <sys/socket.h>
#include <vector>


namespace bg431esc1_actuator {

    using hardware_interface::CallbackReturn;

    CallbackReturn
    Bg431esc1Actuator::on_init(const hardware_interface::HardwareInfo & hardware_info) {
        try {
            for (auto & joint : hardware_info.joints) {
                if (joint.parameters.contains("has_aux_encoder")) {
                    m_bg431esc1_with_aux_data.push_back(BG431ESC1WithAuxData(
                        static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id"))),
                        static_cast<ControlMode>(std::stoi(joint.parameters.at("control_mode"))),
                        joint.name
                    ));
                } else {
                    m_bg431esc1_data.push_back(BG431ESC1Data(
                        static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id"))),
                        static_cast<ControlMode>(std::stoi(joint.parameters.at("control_mode"))),
                        joint.name
                    ));
                }
            }
        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        
        return CallbackReturn::SUCCESS;        
    }

    CallbackReturn 
    Bg431esc1Actuator::on_configure(const rclcpp_lifecycle::State & previous_state) {
        m_socket = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        if (m_socket == -1) return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        if (bind(m_socket, reinterpret_cast<sockaddr *>(&m_address), sizeof(m_address)) == -1) {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Bg431esc1Actuator::on_cleanup(const rclcpp_lifecycle::State & previous_state) {        
        if (close(m_socket) == -1) return CallbackReturn::ERROR;
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    Bg431esc1Actuator::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (BG431ESC1Data datum : m_bg431esc1_data) {
            state_interfaces.push_back(hardware_interface::StateInterface(
                datum.m_name,
                hardware_interface::HW_IF_POSITION,
                &datum.m_position_estimate
            ));
            state_interfaces.push_back(hardware_interface::StateInterface(
                datum.m_name,
                hardware_interface::HW_IF_VELOCITY,
                &datum.m_velocity_estimate
            ));
        }

        for (BG431ESC1WithAuxData datum : m_bg431esc1_with_aux_data) {
            state_interfaces.push_back(hardware_interface::StateInterface(
                datum.m_name + "_output_shaft",
                hardware_interface::HW_IF_POSITION,
                &datum.m_position_estimate
            ));
            state_interfaces.push_back(hardware_interface::StateInterface(
                datum.m_name + "_output_shaft",
                hardware_interface::HW_IF_VELOCITY,
                &datum.m_velocity_estimate
            ));
            state_interfaces.push_back(hardware_interface::StateInterface(
                datum.m_name + "_aux_encoder",
                hardware_interface::HW_IF_POSITION,
                &datum.m_position_estimate
            ));
            state_interfaces.push_back(hardware_interface::StateInterface(
                datum.m_name + "_aux_encoder",
                hardware_interface::HW_IF_VELOCITY,
                &datum.m_velocity_estimate
            ));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    Bg431esc1Actuator::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (BG431ESC1Data datum : m_bg431esc1_data) {
            command_interfaces.push_back(hardware_interface::CommandInterface(
                datum.m_name,
                hardware_interface::HW_IF_POSITION,
                &datum.m_setpoint
            ));
            command_interfaces.push_back(hardware_interface::CommandInterface(
                datum.m_name,
                hardware_interface::HW_IF_VELOCITY,
                &datum.m_setpoint
            ));
        }

        for (BG431ESC1WithAuxData datum : m_bg431esc1_with_aux_data) {
            command_interfaces.push_back(hardware_interface::CommandInterface(
                datum.m_name,
                hardware_interface::HW_IF_POSITION,
                &datum.m_setpoint
            ));
            command_interfaces.push_back(hardware_interface::CommandInterface(
                datum.m_name,
                hardware_interface::HW_IF_VELOCITY,
                &datum.m_setpoint
            ));
        }

        return command_interfaces;
    }

    CallbackReturn
    Bg431esc1Actuator::on_activate(const rclcpp_lifecycle::State & previous_state) {
        enabled = true;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Bg431esc1Actuator::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        enabled = false;
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type  Bg431esc1Actuator::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
        struct canfd_frame frame;
        auto _ = recvfrom(m_socket, &frame, sizeof(canfd_frame),
            0, (struct sockaddr*)&m_address, &m_adress_len);

        CanId id = {.raw = frame.can_id};
        if (id.magic != magic_number) return hardware_interface::return_type::ERROR;

        switch (id.api_page) {
        case 1:
            break; // Loopback, do nothing
        case 2:
            if (id.api_index == 1) { // primary encoder
                for (BG431ESC1Data datum : m_bg431esc1_data) {
                    if (datum.m_can_address == id.address) {
                        motor_controller = 
                    }
                }
            } else { // aux encoder
            }
            break;
        default: // 3
            break;
        }
    }

}