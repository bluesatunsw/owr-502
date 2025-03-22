#include "bg431esc1_actuator/bg431esc1_actuator.hpp"

#include <linux/can.h>
#include <sys/socket.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <mutex>
#include <ranges>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>
#include <thread>

#include "bg431esc1_actuator/can_mux.hpp"

namespace bg431esc1_actuator {
hardware_interface::CallbackReturn Bg431esc1Actuator::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
  if (hardware_interface::ActuatorInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  m_ratio = std::stoul(hardware_info.hardware_parameters.at("ratio"));
  m_use_auxilary =
      hardware_info.hardware_parameters.at("use_auxilary") == "true";

  unsigned long dev_index = std::stoul(hardware_info.hardware_parameters.at("device_index"));
  m_device = CanMux::get_instance().open(kClassId, dev_index);
  m_device.bind(std::bind_front(&Bg431esc1Actuator::recv_callback, this));


  if (hardware_info.joints.size() != 1) {
    RCLCPP_FATAL(get_logger(), "Motor has %zu joints. 1 expected.",
                 hardware_info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto joint{hardware_info.joints.at(0)};
  if (kValidStateInterfaces.size() != joint.state_interfaces.size() ||
      !std::ranges::views::all(
          std::ranges::views::transform(joint.state_interfaces, [](auto i) {
            return std::ranges::contains(kValidStateInterfaces, i.name);
          }))) {
    RCLCPP_FATAL(get_logger(),
                 "Joint '%s' has an invalid set of state interfaces.",
                 joint.name.c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }
  if (kValidCommandInterfaces.size() != joint.command_interfaces.size() ||
      !std::ranges::views::all(
          std::ranges::views::transform(joint.command_interfaces, [](auto i) {
            return std::ranges::contains(kValidCommandInterfaces, i.name);
          }))) {
    RCLCPP_FATAL(get_logger(),
                 "Joint '%s' has an invalid set of command interfaces.",
                 joint.name.c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_logger(), "Motor \"%s\" id %ld, with %s auxiliary encoder successfully configured", 
		  joint.name.c_str(), 
		  dev_index, 
		  m_use_auxilary ? "an" : "no"
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Bg431esc1Actuator::on_shutdown(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  m_device = CanMux::Connection{};

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Bg431esc1Actuator::on_configure(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  for (const auto& [name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto& [name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }

  // Block transistions to the active state until the offset has been calculated
  while (true) {
    {
      std::scoped_lock guard{m_device.mutex()};
      if (m_use_auxilary) {
        if (m_recieved_primary && m_recieved_auxilary) {
          // Set the current position to the auxilary encoder
          m_offset = m_raw_state.primary_position -
                     m_raw_state.auxilary_position * m_ratio;
          break;
        }

      } else {
        if (m_recieved_primary) {
          // Assume that the current position is zero
          m_offset = m_raw_state.primary_position;
          break;
        }
      }
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Waiting for motor to come up...");
    std::this_thread::yield();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Bg431esc1Actuator::on_deactivate(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  m_control_mode = ControlMode::kDisabled;
  // TODO: consider if the device should re-zero themselves
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Bg431esc1Actuator::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  if (stop_interfaces.size() != 0) {
    set_command(stop_interfaces.at(0), 0);
  }

  if (start_interfaces.size() > 1) {
    RCLCPP_FATAL(get_logger(), "Too many command interfaces used at once.");

    return hardware_interface::return_type::ERROR;
  }

  if (start_interfaces.size() != 0) {
    auto start_mode{start_interfaces.at(0)};
    if (start_mode == hardware_interface::HW_IF_POSITION) {
      m_control_mode = ControlMode::kPosition;
    } else if (start_mode == hardware_interface::HW_IF_VELOCITY) {
      m_control_mode = ControlMode::kVelocity;
    } else if (start_mode == hardware_interface::HW_IF_TORQUE) {
      m_control_mode = ControlMode::kTorque;
    } else {
      RCLCPP_FATAL(get_logger(), "Invalid command mode %s.",
                   start_mode.c_str());

      return hardware_interface::return_type::ERROR;
    }
  } else {
    m_control_mode = ControlMode::kDisabled;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Bg431esc1Actuator::read(
    [[maybe_unused]] const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period) {
  // Protect m_incoming_state
  std::scoped_lock guard{m_device.mutex()};

  set_state(std::format("{}/{}", info_.joints[0].name, kPosition),
            (m_raw_state.primary_position - m_offset) / m_ratio);
  set_state(std::format("{}/{}", info_.joints[0].name, kVelocity),
            m_raw_state.primary_velocity / m_ratio);

  set_state(std::format("{}/{}", info_.joints[0].name, kPrimaryPosition),
            m_raw_state.primary_position);
  set_state(std::format("{}/{}", info_.joints[0].name, kPrimaryVelocity),
            m_raw_state.primary_velocity);
  set_state(std::format("{}/{}", info_.joints[0].name, kAuxilaryPosition),
            m_raw_state.auxilary_position);
  set_state(std::format("{}/{}", info_.joints[0].name, kAuxilaryVelocity),
            m_raw_state.auxilary_velocity);

  set_state(std::format("{}/{}", info_.joints[0].name, kAppliedVoltage),
            m_raw_state.applied_voltage * 0.1);
  set_state(std::format("{}/{}", info_.joints[0].name, kStatorCurrent),
            m_raw_state.stator_current * 0.1);
  set_state(std::format("{}/{}", info_.joints[0].name, kSupplyCurrent),
            m_raw_state.supply_current * 0.1);
  set_state(std::format("{}/{}", info_.joints[0].name, kBusVoltage),
            m_raw_state.bus_voltage * 0.01);
  set_state(std::format("{}/{}", info_.joints[0].name, kDriverTemperature),
            m_raw_state.driver_temperature * 0.01);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Bg431esc1Actuator::write(
    [[maybe_unused]] const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period) {
  struct [[gnu::packed]] ControlFrame {
    ControlMode control_mode;
    float setpoint;
  } frame{m_control_mode, 0.0f};

  switch (m_control_mode) {
    case ControlMode::kPosition:
      frame.setpoint = static_cast<float>(get_command(
                           std::format("{}/{}", info_.joints[0].name,
                                       hardware_interface::HW_IF_POSITION))) *
                           m_ratio +
                       m_offset;
      break;
    case ControlMode::kVelocity:
      frame.setpoint = static_cast<float>(get_command(
                           std::format("{}/{}", info_.joints[0].name,
                                       hardware_interface::HW_IF_VELOCITY))) *
                       m_ratio;
      break;
    case ControlMode::kTorque:
      // Velocity and torque are inversely proportional
      frame.setpoint = static_cast<float>(get_command(
                           std::format("{}/{}", info_.joints[0].name,
                                       hardware_interface::HW_IF_TORQUE))) /
                       m_ratio;
      break;
    case ControlMode::kDisabled:
      frame.setpoint = 0.0f;
      break;
    default:
      RCLCPP_FATAL(get_logger(), "Invalid command mode %hhu.",
                   static_cast<std::uint8_t>(m_control_mode));
  }
  m_device.send(kControlFrameIndex, frame,
                m_control_mode == ControlMode::kDisabled
                    ? CanId::Priority::kFast
                    : CanId::Priority::kNominal);

  return hardware_interface::return_type::OK;
}

void Bg431esc1Actuator::recv_callback(CanId::ApiIndex api_index,
                                      std::span<const std::byte> payload) {
  struct [[gnu::packed]] Primary {
    float position;
    float velocity;
  };

  struct [[gnu::packed]] Auxilary {
    float position;
    float velocity;
  };

  struct [[gnu::packed]] Miscellaneous {
    std::int16_t applied_voltage;
    std::int16_t stator_current;
    std::int16_t supply_current;
    std::uint8_t bus_voltage;
    std::uint8_t driver_temperature;
  };

  // Within this context, m_incoming_state is protected by m_device.m_mutex
  switch (api_index) {
    case kPrimaryFrameIndex: {
      auto frame{from_bytes<Primary>(payload)};
      if (!frame) {
        RCLCPP_ERROR(get_logger(),
                     "Incoming primary CAN frame has size %zu. Expected %zu.",
                     payload.size(), sizeof(Primary));
        return;
      }
      m_raw_state.primary_position = frame->position;
      m_raw_state.primary_velocity = frame->velocity;
      m_recieved_primary = true;
      return;
    }

    case kAuxiliaryFrameIndex: {
      auto frame{from_bytes<Auxilary>(payload)};
      if (!frame) {
        RCLCPP_ERROR(get_logger(),
                     "Incoming auxilary CAN frame has size %zu. Expected %zu.",
                     payload.size(), sizeof(Primary));
        return;
      }
      m_raw_state.auxilary_position = frame->position;
      m_raw_state.auxilary_velocity = frame->velocity;
      m_recieved_auxilary = true;
      return;
    }

    case kMiscellaneousFrameIndex: {
      auto frame{from_bytes<Miscellaneous>(payload)};
      if (!frame) {
        RCLCPP_ERROR(
            get_logger(),
            "Incoming miscellaneous CAN frame has size %zu. Expected %zu.",
            payload.size(), sizeof(Primary));
        return;
      }
      m_raw_state.applied_voltage = frame->applied_voltage;
      m_raw_state.stator_current = frame->supply_current;
      m_raw_state.supply_current = frame->supply_current;
      m_raw_state.bus_voltage = frame->bus_voltage;
      m_raw_state.driver_temperature = frame->driver_temperature;
      return;
    }

    default: {
      RCLCPP_ERROR(get_logger(),
                   "Incoming CAN frame (size %zu) has unknown api index %hu.",
                   payload.size(), api_index);
      return;
    }
  }
}
}  // namespace bg431esc1_actuator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bg431esc1_actuator::Bg431esc1Actuator,
                       hardware_interface::ActuatorInterface)
