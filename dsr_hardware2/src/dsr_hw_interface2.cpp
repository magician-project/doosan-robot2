// /*
//  *  Inferfaces for doosan robot controllor
//   * Author: Minsoo Song(minsoo.song@doosan.com)
//  *
//  * Copyright (c) 2024 Doosan Robotics
//  * Use of this source code is governed by the BSD, see LICENSE
// */

#include "dsr_hardware2/dsr_hw_interface2.h"

#include "hardware_interface/hardware_info.hpp"

using dsr_hardware2::DRHWInterface;

using CallbackReturn = dsr_hardware2::DRHWInterface::CallbackReturn;
using return_type    = dsr_hardware2::DRHWInterface::return_type;

CallbackReturn
DRHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
export_state_interfaces() {}

std::vector<hardware_interface::CommandInterface>
export_command_interfaces() {}

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
