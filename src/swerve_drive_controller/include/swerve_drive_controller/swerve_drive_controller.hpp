#ifndef SWERVE_DRIVE_CONTROLLER_HPP
#define SWERVE_DRIVE_CONTROLLER_HPP

#include "visibility_control.h"
#include "controller_interface/controller_interface.hpp"

namespace swerve_drive_controller {

class SwerveDriveController : public controller_interface::ControllerInterface {
    public:
        SWERVE_DRIVE_CONTROLLER_PUBLIC
        SwerveDriveController();

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;

        SWERVE_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;
}

}

#endif