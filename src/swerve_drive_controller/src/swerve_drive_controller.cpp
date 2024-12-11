#include "swerve_drive_controller/swerve_drive_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "swerve_drive_controller_parameters.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <complex>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <tuple>
#include <ranges>

namespace swerve_drive_controller
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using geometry_msgs::msg::TwistStamped;
using namespace std::literals;

CallbackReturn SwerveDriveController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & delta_time)
{
  auto logger = get_node()->get_logger();

  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    // TODO: Halt
    return controller_interface::return_type::OK;
  }

  // TODO: Get current joint states (for odom / optimization)

  // TODO: Calculate and publish Odometry

  // Get latest twistStamped -> run inverse kinematics -> set desired states
  std::shared_ptr<TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  // TODO: Timeout logic

  if (last_command_msg == nullptr) {
    RCLCPP_ERROR(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto chassis_speeds = Eigen::Vector3d{
    last_command_msg->twist.linear.x,
    last_command_msg->twist.linear.y,
    last_command_msg->twist.angular.z
  };

  // Calculate and set inverse kinematics -> wheel speeds (m/s) -> (rad/s)
  const Eigen::VectorXcd moduleStateReqs =
    kinematics_ * chassis_speeds * (1 / params_.wheel_radius);

  // TODO: Desaturate wheel speeds

  // TODO: Swerve optimization
  // TODO: also optimize for reachability

  // TODO: Angle limiting

  // FIXME: check if the order of command_interfaces_ is fixed to be the same as the request from
  // command_interface_configuration
  using namespace std::ranges;
  for (const auto & [state, idx] : views::zip(moduleStateReqs, views::iota(std::size_t{0}))) {
    const bool val_set_err = command_interfaces_[idx * 2].set_value(std::abs(state)) &&
      command_interfaces_[idx * 2 + 1].set_value(std::arg(state));

    RCLCPP_ERROR_EXPRESSION(logger, !val_set_err,
                            "Setting values to command interfaces has failed! "
                            "This means that you are maybe blocking the interface in your hardware for too long.");
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SwerveDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  params_ = param_listener_->get_params();
  RCLCPP_INFO(logger, "Parameters were updated");

  kinematics_ = {};
  kinematics_.resize(static_cast<Eigen::Index>(params_.modules.module_names_map.size()),
      Eigen::NoChange_t{});
  int count = 0;
  for (const auto & [name, module] : params_.modules.module_names_map) {
    kinematics_.template block<1, 3>(count, 0) << 1.0, 1.0i,
      std::complex<double>{-module.translation[1], module.translation[0]};
    count++;
  }

  RCLCPP_INFO(logger, "%d Modules have been configured for kinematics", count);

  const TwistStamped empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<TwistStamped>(empty_twist));


  velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<TwistStamped> msg) -> void {
      // TODO: if (!subscriber_is_active_) {
      //  RCLCPP_WARN(get_node()->get_logger(), "Can't accept new
      //  commands. subscriber is inactive"); return;
      // }      
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
        RCLCPP_WARN_ONCE(get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current"
          " time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }
      received_velocity_msg_ptr_.set(std::move(msg));
    });

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration SwerveDriveController::command_interface_configuration() const
{
  InterfaceConfiguration command_ifaces_config;
  command_ifaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_ifaces_config.names.reserve(params_.module_names.size());
  for (const auto & [mname, module] : params_.modules.module_names_map) {
    command_ifaces_config.names.push_back(module.drive_joint + "/" +
        hardware_interface::HW_IF_VELOCITY);
    command_ifaces_config.names.push_back(module.steering_joint + "/" +
        hardware_interface::HW_IF_POSITION);
  }

  return command_ifaces_config;
}

InterfaceConfiguration SwerveDriveController::state_interface_configuration() const
{
  InterfaceConfiguration state_ifaces_config;
  state_ifaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_ifaces_config.names.reserve(params_.module_names.size());
  for (const auto & [mname, module] : params_.modules.module_names_map) {
    state_ifaces_config.names.push_back(module.drive_joint + "/" +
        hardware_interface::HW_IF_VELOCITY);
    state_ifaces_config.names.push_back(module.steering_joint + "/" +
        hardware_interface::HW_IF_POSITION);
  }

  return state_ifaces_config;
}

// TODO:
CallbackReturn SwerveDriveController::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

// TODO:
CallbackReturn SwerveDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

// TODO:
CallbackReturn SwerveDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

// TODO:
CallbackReturn SwerveDriveController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

// TODO:
CallbackReturn SwerveDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}
}  // namespace swerve_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(swerve_drive_controller::SwerveDriveController,
  controller_interface::ControllerInterface)
