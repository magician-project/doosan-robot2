/*********************************************************************
 *
 *  Inferfaces for doosan robot controllor
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Doosan Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <rclcpp/logger.hpp>
#define _DEBUG_DSR_CTL 1

#define USE_FULL_LIB

#ifndef DSR_HARDWARE2__DR_HW_INTERFACE2_H
#define DSR_HARDWARE2__DR_HW_INTERFACE2_H

#include <Eigen/Dense>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace dsr_hardware2 {

class HARDWARE_INTERFACE_PUBLIC DRHWInterface
        : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;

    using CallbackReturn =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // ~DRHWInterface();

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces(
    ) override;

    hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
            override;

protected:
    rclcpp::Logger
    get_logger() const {
        return rclcpp::get_logger("dsr_hw_interface2");
    }

    enum ControlMode {
        NOT_SET = 0,
        POSITION,
        VELOCITY,
        TORQUE
    } _control_mode;

    using Vec6double = Eigen::Vector<double, 6>;
    using Vec6float  = Eigen::Vector<float, 6>;

    Vec6double _q_state;
    Vec6double _q_dot_state;
    Vec6double _tau_state;
    Vec6double _q_cmd;
    Vec6double _q_dot_cmd;
    Vec6double _tau_cmd;

    Vec6float _q_dot_limit;
    Vec6float _q_dot_dot_limit;
    Vec6float _tau_grav;

    const std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
};


}  // namespace dsr_hardware2

#endif  // end
