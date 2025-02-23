// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tb3_hw_interface/tb3_system.hpp"
#include "tb3_hw_interface/arduino_comms.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Using modified code from joshnewans articubot-one
namespace tb3_hw_interface {
    hardware_interface::CallbackReturn Tb3SystemHardware::on_init(const hardware_interface::HardwareInfo & info) {

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        config_.left_wheel = info_.hardware_parameters["left_wheel_name"];
        config_.right_wheel = info_.hardware_parameters["right_wheel_name"];
        config_.device = info_.hardware_parameters["device"];
        config_.baud_rate = hardware_interface::stod(info_.hardware_parameters["baud_rate"]);
        config_.timeout_ms = hardware_interface::stod(info_.hardware_parameters["timeout_ms"]);
        config_.enc_counts_per_rev = hardware_interface::stod(info_.hardware_parameters["enc_counts_per_rev"]);

        wheel_l_.setup(config_.left_wheel, config_.enc_counts_per_rev);
        wheel_r_.setup(config_.right_wheel, config_.enc_counts_per_rev);

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            // System has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Tb3SystemHardware::on_configure(const rclcpp_lifecycle::State &) {
        // reset values always when configuring hardware
        for (const auto & [name, descr] : joint_state_interfaces_) {
            set_state(name, 0.0);
        }
        for (const auto & [name, descr] : joint_command_interfaces_) {
            set_command(name, 0.0);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Tb3SystemHardware::on_activate(const rclcpp_lifecycle::State &) {
        comms_.connect(config_.device, config_.baud_rate, config_.timeout_ms);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Tb3SystemHardware::on_deactivate(const rclcpp_lifecycle::State &) {
        comms_.disconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Tb3SystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period) {
        comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

        double delta_seconds = period.seconds();

        double pos_prev = get_state("wheel_left_joint/position");
        wheel_l_.pos = wheel_l_.calc_enc_angle();
        set_state("wheel_left_joint/position", wheel_l_.pos);
        wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;
        set_state("wheel_left_joint/velocity", wheel_l_.vel);

        pos_prev = get_state("wheel_right_joint/position");
        wheel_r_.pos = wheel_r_.calc_enc_angle();
        set_state("wheel_right_joint/position", wheel_r_.pos);
        wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;
        set_state("wheel_right_joint/velocity", wheel_r_.vel);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type tb3_hw_interface ::Tb3SystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
        int motor_l_counts_per_sec =  get_command("wheel_left_joint/velocity") / wheel_l_.rads_per_count;
        int motor_r_counts_per_sec = get_command("wheel_right_joint/velocity") / wheel_r_.rads_per_count;
        comms_.set_motor_values(motor_l_counts_per_sec, motor_r_counts_per_sec);

        return hardware_interface::return_type::OK;
    }

}  // namespace tb3_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  tb3_hw_interface::Tb3SystemHardware, hardware_interface::SystemInterface)
