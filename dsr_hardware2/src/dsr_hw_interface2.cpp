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
#include <iostream>
#include <thread>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

#include "DRFC.h"
#include "DRFLEx.h"

using dsr_hardware2::DRHWInterface;
using std::cout, std::endl;

using CallbackReturn = dsr_hardware2::DRHWInterface::CallbackReturn;
using return_type    = dsr_hardware2::DRHWInterface::return_type;

// Global variables
DRAFramework::CDRFLEx drfl;
bool                  has_control_authority       = false;
bool                  tp_initialisation_completed = false;

// Configuration variables
const std::string  host_ip       = "192.168.127.100";
const unsigned int host_tcp_port = 12345;
const unsigned int host_udp_port = 12347;

constexpr float None = -10000;

// Callback functions declaration
namespace callback {
// Callbacks that do not modify the global state
void on_tp_popup(LPMESSAGE_POPUP popup);
void on_tp_log(const char* strLog);
void on_tp_progress(LPMESSAGE_PROGRESS progress);
void on_tp_get_user_input(LPMESSAGE_INPUT input);
void on_homing_completed();
void on_monitoring_data(const LPMONITORING_DATA pData);
void on_monitoring_data_exchange(const LPMONITORING_DATA_EX pData);
void on_monitoring_ctrl_io(const LPMONITORING_CTRLIO pData);
void on_monitoring_ctrl_io_exchange(const LPMONITORING_CTRLIO_EX pData);
void on_log_alarm(LPLOG_ALARM tLog);
void on_program_stopped(const PROGRAM_STOP_CAUSE);

// Callbacks that can modify the global state
void on_tp_initialising_completed();
void on_monitoring_state(const ROBOT_STATE);
void on_monitoring_access_control(const MONITORING_ACCESS_CONTROL control);
}  // namespace callback

/* NOTE
 * The only callback missing is the one for RT data stream input.
 */

// Conversion functions
auto double_to_float = [](const double& data) -> float {
    return static_cast<float>(data);
};

auto float_to_double = [](const float& data) -> double {
    return static_cast<double>(data);
};

//  _   _               _
// | | | | __ _ _ __ __| |_      ____ _ _ __ ___
// | |_| |/ _` | '__/ _` \ \ /\ / / _` | '__/ _ \
// |  _  | (_| | | | (_| |\ V  V / (_| | | |  __/
// |_| |_|\__,_|_|  \__,_| \_/\_/ \__,_|_|  \___|
//
//  ___       _             __
// |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
//  | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
//  | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

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


    // Register callbacks (non-modifying)
    drfl.set_on_homming_completed(callback::on_homing_completed);
    drfl.set_on_monitoring_data(callback::on_monitoring_data);
    drfl.set_on_monitoring_data_ex(callback::on_monitoring_data_exchange);
    drfl.set_on_monitoring_ctrl_io(callback::on_monitoring_ctrl_io);
    drfl.set_on_monitoring_ctrl_io_ex(callback::on_monitoring_ctrl_io_exchange);
    drfl.set_on_log_alarm(callback::on_log_alarm);
    drfl.set_on_tp_popup(callback::on_tp_popup);
    drfl.set_on_tp_log(callback::on_tp_log);
    drfl.set_on_tp_progress(callback::on_tp_progress);
    drfl.set_on_tp_get_user_input(callback::on_tp_get_user_input);
    drfl.set_on_program_stopped(callback::on_program_stopped);

    // Register callbacks (the following CAN modify the global state)
    drfl.set_on_monitoring_state(callback::on_monitoring_state);
    drfl.set_on_monitoring_access_control(callback::on_monitoring_access_control);
    drfl.set_on_tp_initializing_completed(callback::on_tp_initialising_completed);


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
    drfl.set_robot_control(CONTROL_SERVO_ON);
    drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
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

//   ____      _ _ _                _
//  / ___|__ _| | | |__   __ _  ___| | _____
// | |   / _` | | | '_ \ / _` |/ __| |/ / __|
// | |__| (_| | | | |_) | (_| | (__|   <\__ \
//  \____\__,_|_|_|_.__/ \__,_|\___|_|\_\___/
//
void
callback::on_tp_popup(LPMESSAGE_POPUP tPopup) {
    cout << "Popup Message: " << tPopup->_szText << endl;
    cout << "Message Level: " << tPopup->_iLevel << endl;
    cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void
callback::on_tp_log(const char* strLog) {
    cout << "Log Message: " << strLog << endl;
}

void
callback::on_tp_progress(LPMESSAGE_PROGRESS tProgress) {
    cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
    cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void
callback::on_tp_get_user_input(LPMESSAGE_INPUT tInput) {
    cout << "User Input : " << tInput->_szText << endl;
    cout << "Data Type : " << (int)tInput->_iType << endl;
}

void
callback::on_homing_completed() {
    cout << "homing completed" << endl;
}

void
callback::on_monitoring_data(const LPMONITORING_DATA pData) {
    return;
    cout << "# monitoring 0 data " << pData->_tCtrl._tTask._fActualPos[0][0]
         << pData->_tCtrl._tTask._fActualPos[0][1]
         << pData->_tCtrl._tTask._fActualPos[0][2]
         << pData->_tCtrl._tTask._fActualPos[0][3]
         << pData->_tCtrl._tTask._fActualPos[0][4]
         << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void
callback::on_monitoring_data_exchange(const LPMONITORING_DATA_EX pData) {
    return;
    cout << "# monitoring 1 data " << pData->_tCtrl._tWorld._fTargetPos[0]
         << pData->_tCtrl._tWorld._fTargetPos[1] << pData->_tCtrl._tWorld._fTargetPos[2]
         << pData->_tCtrl._tWorld._fTargetPos[3] << pData->_tCtrl._tWorld._fTargetPos[4]
         << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void
callback::on_monitoring_ctrl_io(const LPMONITORING_CTRLIO pData) {
    return;
    cout << "# monitoring ctrl 0 data" << endl;
    for (int i = 0; i < 16; i++) { cout << (int)pData->_tInput._iActualDI[i] << endl; }
}

void
callback::on_monitoring_ctrl_io_exchange(const LPMONITORING_CTRLIO_EX pData) {
    return;
    cout << "# monitoring ctrl 1 data" << endl;
    for (int i = 0; i < 16; i++) { cout << (int)pData->_tInput._iActualDI[i] << endl; }
    for (int i = 0; i < 16; i++) { cout << (int)pData->_tOutput._iTargetDO[i] << endl; }
}

void
callback::on_log_alarm(LPLOG_ALARM tLog) {
    cout << "Alarm Info: "
         << "group(" << (unsigned int)tLog->_iGroup << "), index(" << tLog->_iIndex
         << "), param(" << tLog->_szParam[0] << "), param(" << tLog->_szParam[1]
         << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void
callback::on_program_stopped(const PROGRAM_STOP_CAUSE cause) {
    cout << "Program stopped (cause " << cause << ")" << endl;
}

void
callback::on_tp_initialising_completed() {
    tp_initialisation_completed = true;
    drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void
callback::on_monitoring_state(const ROBOT_STATE state) {
    switch ((unsigned char)state) {
        case STATE_EMERGENCY_STOP:
            // popup
            break;
        case STATE_STANDBY:
        case STATE_MOVING:
        case STATE_TEACHING:
            break;
        case STATE_SAFE_STOP:
            if (has_control_authority) {
                drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
                drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
            }
            break;
        case STATE_SAFE_OFF:
            cout << "State STATE_SAFE_OFF\n";
            if (has_control_authority) { drfl.SetRobotControl(CONTROL_SERVO_ON); }
            break;
        case STATE_SAFE_STOP2:
            cout << "State STATE_SAFE_STOP2\n";
            if (has_control_authority) {
                drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
            }
            break;
        case STATE_SAFE_OFF2:
            cout << "State STATE_SAFE_OFF2\n";
            if (has_control_authority) {
                drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
            }
            break;
        case STATE_RECOVERY:
            // Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
            break;
        default:
            break;
    }
    return;
}

void
callback::on_monitoring_access_control(const MONITORING_ACCESS_CONTROL control) {
    switch (control) {
        case MONITORING_ACCESS_CONTROL_REQUEST:
            assert(drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
            // Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
            break;
        case MONITORING_ACCESS_CONTROL_GRANT:
            cout << "Access control grant" << endl;
            has_control_authority = TRUE;
            callback::on_monitoring_state(drfl.GetRobotState());
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
        case MONITORING_ACCESS_CONTROL_LOSS:
            cout << "Access control revoked" << endl;
            has_control_authority = FALSE;
            if (tp_initialisation_completed) {
                drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            }
            break;
        default:
            break;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface
)
