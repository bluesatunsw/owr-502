#include "swerve_drive_controller/swerve_drive_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <complex>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/qos.hpp>

namespace swerve_drive_controller {
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";

    using controller_interface::CallbackReturn;
    using geometry_msgs::msg::TwistStamped;
    using namespace std::literals;

    CallbackReturn SwerveDriveController::on_init() {
        try {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type SwerveDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & delta_time) {
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

        const auto chassis_speeds = Eigen::Vector3d{
            last_command_msg->twist.linear.x,
            last_command_msg->twist.linear.y,
            last_command_msg->twist.angular.z
        };

        const Eigen::VectorXd moduleStateReqs = kinematics_ * chassis_speeds;
        for (int i = 0; i < kinematics_.rows() / 2; i++) {
            double x{moduleStateReqs(i * 2, 0)};
            double y{moduleStateReqs(i * 2 + 1, 0)};
        }
        
        // FIXME: Calculate and set inverse kinematics -> wheel speeds

        // TODO: Desaturate wheel speeds
        // TODO: Swerve optimization
        // TODO: Angle limiting

        return controller_interface::return_type::OK;
    }

    CallbackReturn SwerveDriveController::on_configure(const rclcpp_lifecycle::State & previous_state) {
        auto logger = get_node()->get_logger();
 
        // update parameters if they have changed
        if (!param_listener_->is_old(params_)) {
            RCLCPP_INFO(logger, "Skipped updating same parameters");
            return CallbackReturn::SUCCESS;
        }

        params_ = param_listener_->get_params();
        RCLCPP_INFO(logger, "Parameters were updated");

        kinematics_ = {};
        int count = 0;
        for (const auto& [name, module] : params_.kinematics.module_names_map) {
            kinematics_.template block<1,3>(count, 0) << 
                1.0, 1.0i, std::complex<double>{-module.translation[1], module.translation[0]};
            count++;

        }


        last_command_msg_ = std::make_shared<TwistStamped>();

        get_node()->create_subscription<TwistStamped>(
            DEFAULT_COMMAND_TOPIC, 
            rclcpp::SystemDefaultsQoS(), 
            [this](const std::shared_ptr<TwistStamped> msg) -> void {
                // TODO: if (!subscriber_is_active_) {
                //     RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                //     return;
                // }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                    RCLCPP_WARN_ONCE(
                    get_node()->get_logger(),
                    "Received TwistStamped with zero timestamp, setting it to current "
                    "time, this message will only be shown once");
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                received_velocity_msg_ptr_.set(std::move(msg));
            }
        );

        return CallbackReturn::SUCCESS;
    }
}