// /*
//  *  Inferfaces for doosan robot controllor
//   * Author: Minsoo Song(minsoo.song@doosan.com)
//  *
//  * Copyright (c) 2024 Doosan Robotics
//  * Use of this source code is governed by the BSD, see LICENSE
// */

#include "dsr_hardware2/dsr_hw_interface2.h"

#include <chrono>
#include <DRFC.h>
#include <thread>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

#include "DRFLEx.h"

using dsr_hardware2::DRHWInterface;

using CallbackReturn = dsr_hardware2::DRHWInterface::CallbackReturn;
using return_type    = dsr_hardware2::DRHWInterface::return_type;

// Global variables
DRAFramework::CDRFLEx drfl;
bool                  has_control_authority       = FALSE;
bool                  tp_initialisation_completed = FALSE;

// Configuration variables
const std::string  host_ip       = "192.168.127.100";
const unsigned int host_tcp_port = 12345;
const unsigned int host_udp_port = 12347;
CallbackReturn
DRHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Parent 'on_init()' function call failed!");
        return CallbackReturn::ERROR;
    }

    // Initiliase member variables
    _q_state     = Vec6double::Zero();
    _q_dot_state = Vec6double::Zero();
    _tau_state   = Vec6double::Zero();
    _q_cmd       = Vec6double::Zero();
    _q_dot_cmd   = Vec6double::Zero();
    _tau_cmd     = Vec6double::Zero();

    _q_dot_limit     = 70 * Vec6float::Ones();
    _q_dot_dot_limit = 70 * Vec6float::Ones();


    // TODO: register callback here


    // Communication initialisation and robot setup
    RCLCPP_INFO(
            get_logger(),
            "Connecting to robot on IP %s:%u",
            host_ip.c_str(),
            host_tcp_port
    );
    if (!drfl.open_connection(host_ip, host_tcp_port)) {
        RCLCPP_ERROR(get_logger(), "Unable to establish connection with robot");
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Connected!");

    SYSTEM_VERSION sys_version = {
            '\0',
    };
    if (!drfl.get_system_version(&sys_version)) return CallbackReturn::ERROR;
    if (!drfl.setup_monitoring_version(1)) return CallbackReturn::ERROR;
    RCLCPP_INFO(get_logger(), "System version: %s", sys_version._szController);
    RCLCPP_INFO(get_logger(), "Libray version: %s", drfl.get_library_version());

    while ((drfl.get_robot_state() != STATE_STANDBY) || !has_control_authority) {
        RCLCPP_DEBUG(
                get_logger(),
                "Waiting robot to reach standby state, or that program has control "
                "authority"
        );
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const bool is_emulator = host_ip == "127.0.0.1";
    if (is_emulator) { RCLCPP_INFO(get_logger(), "Running in emulator mode"); }
    if (!drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
        RCLCPP_ERROR(get_logger(), "Unable to set robot in autonomous mode");
        return CallbackReturn::FAILURE;
    }
    const ROBOT_SYSTEM sys = is_emulator ? ROBOT_SYSTEM_VIRTUAL : ROBOT_SYSTEM_REAL;
    if (!drfl.set_robot_system(sys)) {
        RCLCPP_ERROR(get_logger(), "Unable to set proper robot system (virtual/real)");
        return CallbackReturn::FAILURE;
    }

    // Real-time communication initialisation and setup
    RCLCPP_INFO(
            get_logger(),
            "Connecting RT control on %s:%u",
            host_ip.c_str(),
            host_udp_port
    );
    if (!drfl.connect_rt_control(host_ip, host_udp_port)) {
        RCLCPP_ERROR(get_logger(), "Unable to connect RT control stream");
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Connected!");

    const std::string version   = "v1.0";
    const float       period    = 0.001;
    const int         losscount = 4;
    if (!drfl.set_rt_control_output(version, period, losscount)) {
        RCLCPP_ERROR(get_logger(), "Unable to connect RT control stream");
        return CallbackReturn::FAILURE;
    }

    if (!drfl.start_rt_control()) {
        RCLCPP_ERROR(get_logger(), "Unable to start RT control");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Setting velocity and acceleration limits");
    if (!drfl.set_velj_rt(_q_dot_limit.data())) return CallbackReturn::ERROR;
    if (!drfl.set_accj_rt(_q_dot_dot_limit.data())) return CallbackReturn::ERROR;
    RCLCPP_DEBUG_STREAM(
            get_logger(), "Joint velocity limit: " << _q_dot_limit.transpose()
    );
    RCLCPP_DEBUG_STREAM(
            get_logger(), "Joint acceleration limit: " << _q_dot_dot_limit.transpose()
    );

    drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

    RCLCPP_INFO(get_logger(), "DSR hardware interface initialisation completed");
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
