// /*
//  *  Inferfaces for doosan robot controllor
//   * Author: Minsoo Song(minsoo.song@doosan.com)
//  *
//  * Copyright (c) 2024 Doosan Robotics
//  * Use of this source code is governed by the BSD, see LICENSE
// */

#include "dsr_hardware2/dsr_hw_interface2.h"

#include <iterator>

#include <rclcpp/logging.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using dsr_hardware2::DRHWInterface;

using CallbackReturn = dsr_hardware2::DRHWInterface::CallbackReturn;
using return_type    = dsr_hardware2::DRHWInterface::return_type;

CallbackReturn
DRHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DRHWInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;

    RCLCPP_INFO(get_logger(), "Registering position state interfaces");
    for (int i{0}; i < 6; ++i) {
        RCLCPP_DEBUG(get_logger(), "Registering joint %s", joint_names.at(i).c_str());
        interfaces.emplace_back(
                joint_names.at(i), hardware_interface::HW_IF_POSITION, &_q_state(i)
        );
    }

    RCLCPP_INFO(get_logger(), "Registering velocity state interfaces");
    for (int i{0}; i < 6; ++i) {
        RCLCPP_DEBUG(get_logger(), "Registering joint %s", joint_names.at(i).c_str());
        interfaces.emplace_back(
                joint_names.at(i), hardware_interface::HW_IF_VELOCITY, &_q_dot_state(i)
        );
    }

    RCLCPP_INFO(get_logger(), "Registering effort state interfaces");
    for (int i{0}; i < 6; ++i) {
        RCLCPP_DEBUG(get_logger(), "Registering joint %s", joint_names.at(i).c_str());
        interfaces.emplace_back(
                joint_names.at(i), hardware_interface::HW_IF_EFFORT, &_tau_state(i)
        );
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface>
DRHWInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;

    RCLCPP_INFO(get_logger(), "Registering position command interfaces");
    for (int i{0}; i < 6; ++i) {
        RCLCPP_DEBUG(get_logger(), "Registering joint %s", joint_names.at(i).c_str());
        interfaces.emplace_back(
                joint_names.at(i), hardware_interface::HW_IF_POSITION, &_q_cmd(i)
        );
    }

    RCLCPP_INFO(get_logger(), "Registering velocity command interfaces");
    for (int i{0}; i < 6; ++i) {
        RCLCPP_DEBUG(get_logger(), "Registering joint %s", joint_names.at(i).c_str());
        interfaces.emplace_back(
                joint_names.at(i), hardware_interface::HW_IF_VELOCITY, &_q_dot_cmd(i)
        );
    }

    RCLCPP_INFO(get_logger(), "Registering effort command interfaces");
    for (int i{0}; i < 6; ++i) {
        RCLCPP_DEBUG(get_logger(), "Registering joint %s", joint_names.at(i).c_str());
        interfaces.emplace_back(
                joint_names.at(i), hardware_interface::HW_IF_EFFORT, &_tau_cmd(i)
        );
    }

    return interfaces;
}

hardware_interface::return_type
DRHWInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
) {
    return return_type::OK;
}

return_type
DRHWInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    return return_type::OK;
}

return_type
DRHWInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface
)
