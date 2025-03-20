#ifndef BG431ESC1_ACTUATOR_HPP
#define BG431ESC1_ACTUATOR_HPP

#include <linux/can.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string_view>
#include <vector>

#include "bg431esc1_actuator/can_mux.hpp"
#include "visibility_control.h"

namespace bg431esc1_actuator {
class Bg431esc1Actuator : public hardware_interface::ActuatorInterface {
 public:
  // LifecycleNodeInterface
  BG431ESC1_ACTUATOR_PUBLIC
  Bg431esc1Actuator() = default;

  BG431ESC1_ACTUATOR_PUBLIC
  CallbackReturn on_init(
      const hardware_interface::HardwareInfo& hardware_info) override;

  BG431ESC1_ACTUATOR_PUBLIC
  CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

  BG431ESC1_ACTUATOR_PUBLIC
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  BG431ESC1_ACTUATOR_PUBLIC
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // ActuatorInterface
  BG431ESC1_ACTUATOR_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  BG431ESC1_ACTUATOR_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  BG431ESC1_ACTUATOR_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void recv_callback(CanId::ApiIndex api_index,
                     std::span<const std::byte> payload);

  constexpr static CanId::DeviceClass kClassId = 0x18;
  constexpr static CanId::ApiIndex kControlFrameIndex = 0x401;

  constexpr static CanId::ApiIndex kPrimaryFrameIndex = 0x801;
  constexpr static CanId::ApiIndex kAuxiliaryFrameIndex = 0x802;
  constexpr static CanId::ApiIndex kMiscellaneousFrameIndex = 0x803;

  constexpr static std::array<std::string_view, 3> kValidCommandInterfaces{
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_TORQUE,
  };

  constexpr static std::string_view kPrimaryPosition = "primary_position";
  constexpr static std::string_view kPrimaryVelocity = "primary_velocity";
  constexpr static std::string_view kAuxilaryPosition = "auxilary_position";
  constexpr static std::string_view kAuxilaryVelocity = "auxilary_velocity";

  constexpr static std::string_view kAppliedVoltage = "applied_voltage";
  constexpr static std::string_view kStatorCurrent = "stator_current";
  constexpr static std::string_view kSupplyCurrent = "supply_current";
  constexpr static std::string_view kBusVoltage = "bus_voltage";
  constexpr static std::string_view kDriverTemperature = "driver_temperature";

  constexpr static std::array<std::string_view, 9> kValidStateInterfaces{
      kPrimaryPosition,  kPrimaryVelocity, kAuxilaryPosition,
      kAuxilaryVelocity, kAppliedVoltage,  kStatorCurrent,
      kSupplyCurrent,    kBusVoltage,      kDriverTemperature,
  };

  struct StateData {
    double primary_position{};
    double primary_velocity{};

    double auxilary_position{};
    double auxilary_velocity{};

    double applied_voltage{};
    double stator_current{};
    double supply_current{};
    double bus_voltage{};
    double driver_temperature{};
  } m_state {};

  enum class ControlMode : uint8_t {
    kDisabled = 0,
    kVoltage = 1,
    kTorque = 2,
    kVelocity = 3,
    kPosition = 4,
    kVelocityOpenLoop = 5,
    kPositionOpenLoop = 6,
  } m_control_mode { ControlMode::kDisabled };

  CanMux::Connection m_device;
};
}  // namespace bg431esc1_actuator

#endif  // BG431ESC1_ACTUATOR_HPP
