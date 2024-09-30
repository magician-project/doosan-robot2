// /*
//  *  Inferfaces for doosan robot controllor
//   * Author: Minsoo Song(minsoo.song@doosan.com)
//  *
//  * Copyright (c) 2024 Doosan Robotics
//  * Use of this source code is governed by the BSD, see LICENSE
// */


// TODOS:
// Disconnection procedure
#include "dsr_hardware2/dsr_hw_interface2.h"

#include <chrono>
#include <cmath>
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

constexpr float None = -10000;

// Conversion functions
auto double_to_float = [](const double& data) -> float {
    return static_cast<float>(data);
};

auto float_to_double = [](const float& data) -> double {
    return static_cast<double>(data);
};

CallbackReturn
DRHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Parent 'on_init()' function call failed!");
        return CallbackReturn::ERROR;
    }

    // Initiliase member variables
    _control_mode = NOT_SET;

    _q_state     = Vec6double::Zero();
    _q_dot_state = Vec6double::Zero();
    _tau_state   = Vec6double::Zero();
    _q_cmd       = Vec6double::Zero();
    _q_dot_cmd   = Vec6double::Zero();
    _tau_cmd     = Vec6double::Zero();


    _q_dot_limit     = 70 * Vec6float::Ones();
    _q_dot_dot_limit = 70 * Vec6float::Ones();
    _tau_grav        = Vec6float::Zero();


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
        const std::vector<std::string>& /* stop_interfaces */
) {
    RCLCPP_DEBUG(get_logger(), "Called command mode switch");
    if (start_interfaces.empty()) {
        RCLCPP_DEBUG(get_logger(), "No control mode specified, leaving unchanged!");
        return return_type::OK;
    }

    const std::string req_cmd_interface = start_interfaces[0];
    const std::string mode = req_cmd_interface.substr(req_cmd_interface.find('/') + 1);

    if (mode == hardware_interface::HW_IF_POSITION) {
        RCLCPP_INFO(get_logger(), "Switching to position control");
        _control_mode = POSITION;
    } else if (mode == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO(get_logger(), "Switching to velocity control");
        _control_mode = VELOCITY;
    } else if (mode == hardware_interface::HW_IF_EFFORT) {
        RCLCPP_INFO(get_logger(), "Switching to torque control");
        _control_mode = TORQUE;
    } else {
        RCLCPP_ERROR(get_logger(), "Unknown control mode '%s'", mode.data());
        return hardware_interface::return_type::ERROR;
    }

    return return_type::OK;
}

return_type
DRHWInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    auto deg_to_rad = [](const float& data) -> double { return data * M_PI / 180.0; };

    const LPRT_OUTPUT_DATA_LIST data = drfl.read_data_rt();
    // Joint position
    std::transform(
            data->actual_joint_position,
            data->actual_joint_position + 6,
            _q_state.begin(),
            deg_to_rad
    );
    // Joint velocity
    std::transform(
            data->actual_joint_velocity,
            data->actual_joint_velocity + 6,
            _q_dot_state.begin(),
            deg_to_rad
    );
    // Joint torque
    std::transform(
            data->external_joint_torque,
            data->external_joint_torque + 6,
            _tau_state.begin(),
            float_to_double
    );

    // Gravity compensation torque
    std::transform(
            data->gravity_torque,
            data->gravity_torque + 6,
            _tau_grav.begin(),
            float_to_double
    );
    return return_type::OK;
}

return_type
DRHWInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    auto rad_to_deg = [](const double& data) -> float { return data * 180.0 / M_PI; };

    Vec6float cmd;
    Vec6float empty({None, None, None, None, None, None});
    switch (_control_mode) {
        case NOT_SET:
            break;
        case POSITION:
            std::transform(_q_cmd.begin(), _q_cmd.end(), cmd.data(), rad_to_deg);
            drfl.servoj_rt(cmd.data(), empty.data(), empty.data(), 0.0);
            break;
        case VELOCITY:
            std::transform(
                    _q_dot_cmd.begin(), _q_dot_cmd.end(), cmd.data(), rad_to_deg
            );
            drfl.speedj_rt(cmd.data(), empty.data(), 0.0);
            break;
        case TORQUE:
            // use "empty" as storage for the desired torque w/o gravity compensation
            std::transform(
                    _tau_cmd.begin(), _tau_cmd.end(), empty.data(), double_to_float
            );
            cmd = _tau_grav + empty;
            drfl.torque_rt(cmd.data(), 0.0);
            break;
    }
    return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface
)
