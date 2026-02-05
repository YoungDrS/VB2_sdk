#include "jaka_k1_dual_control/jaka_dual_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "jaka_sdk/jkerr.h"
#include "jaka_sdk/jktypes.h"
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#define DEV_INFO(...)                                                          \
  RCLCPP_INFO(rclcpp::get_logger("JakaDualHardware"), __VA_ARGS__)

#define DEV_FATAL(...)                                                         \
  RCLCPP_INFO(rclcpp::get_logger("JakaDualHardware"), __VA_ARGS__)

namespace jaka_dual_control {
hardware_interface::CallbackReturn
JakaDualHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto &j : info_.joints) {
    DEV_INFO("Joint name: %s", j.name.c_str());
  }
  if (read_configure(info) != hardware_interface::CallbackReturn::SUCCESS) {
    DEV_FATAL("Read configure failed!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (OpenDevice() != hardware_interface::CallbackReturn::SUCCESS) {
    DEV_FATAL("Open device failed!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  DEV_INFO("Open device success!");

  if (info_.joints.size() != 14) {
    RCLCPP_FATAL(rclcpp::get_logger("JakaDualHardware"),
                 "The number of crawlers is set not correactly,please check "
                 "your urdf. 14 is expected,but now it is %zu.",
                 info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("JakaDualHardware"),
                "Found 14 crawlers sucessfully!");
  }

  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("JakaDualHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("JakaDualHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("JakaDualHardware"),
                   "Joint '%s' has %zu state interface. 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("JakaDualHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JakaDualHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    DEV_INFO("**********Exported state interface %s",
             info_.joints[i].name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
JakaDualHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
    DEV_INFO("**********Exported command interface %s",
             info_.joints[i].name.c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn JakaDualHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code

  JointValue jl{}, jr{};
  errno_t left_res = device_handler_->edg_get_stat(0, &jl, nullptr);
  errno_t right_res = device_handler_->edg_get_stat(1, &jr, nullptr);
  if (left_res != ERR_SUCC || right_res != ERR_SUCC) {
    DEV_FATAL("On Activate Read positions failed!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (int i = 0; i < 7; ++i) {
    hw_positions_[i] = jl.jVal[i];
    hw_commands_[i] = jl.jVal[i];
  }
  for (int i = 0; i < 7; ++i) {
    hw_positions_[i + 7] = jr.jVal[i];
    hw_commands_[i + 7] = jr.jVal[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("JakaDualHardware"),
              "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JakaDualHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("JakaDualHardware"),
              "Deactivating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code
  // 根据实际需求查看是否需要如此操作
  device_handler_->servo_move_enable(FALSE, -1);
  // device_handler_->disable_robot();
  // device_handler_->power_off();
  // device_handler_->login_out();
  RCLCPP_INFO(rclcpp::get_logger("JakaDualHardware"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
jaka_dual_control::JakaDualHardware::read(const rclcpp::Time & /*time*/,
                                          const rclcpp::Duration &period) {

  JointValue jl{}, jr{};
  const bool okL = device_handler_->edg_get_stat(0, &jl, nullptr) == 0;
  const bool okR = device_handler_->edg_get_stat(1, &jr, nullptr) == 0;
  if (!okL || !okR) {
    DEV_FATAL("Read positions failed!");
    return hardware_interface::return_type::ERROR;
  }
  std::ostringstream oss;
  for (int i = 0; i < 7; ++i) {
    hw_positions_[i] = jl.jVal[i];
  }
  // DEV_INFO("Read left positions: %s", oss.str().c_str());
  std::ostringstream oss1;
  for (int i = 0; i < 7; ++i) {
    hw_positions_[i + 7] = jr.jVal[i];
  }
  BOOL in_drag = false;
  device_handler_->is_in_drag_mode(&in_drag);
  // DEV_INFO("Is in drag mode: %d", in_drag);
  is_in_drag.store(in_drag);
  BOOL in_collision = false;
  device_handler_->is_in_collision(&in_collision);
  is_in_collision.store(in_collision);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type jaka_dual_control::JakaDualHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  JointValue jl{}, jr{};
  std::ostringstream oss;
  for (int i = 0; i < 7; ++i) {
    jl.jVal[i] = hw_commands_[i];
    //oss << hw_commands_[i] << " ";
  }
  //DEV_INFO("Write left positions: %s", oss.str().c_str());
  std::ostringstream oss1;
  for (int i = 0; i < 7; ++i) {
    jr.jVal[i] = hw_commands_[i + 7];
    //oss1 << hw_commands_[i + 7] << " ";
  }
  //DEV_INFO("Write right positions: %s", oss1.str().c_str());
  if (is_in_collision) {
    DEV_FATAL("Robot in collision, please recover first!");
    return hardware_interface::return_type::ERROR;
  }
  if (is_in_drag) {
    DEV_FATAL("Robot in drag mode, please exit drag mode first!");
    return hardware_interface::return_type::OK;
  }
#if 1
  int retL = device_handler_->edg_servo_j(0u, &jl, MoveMode::ABS);
  int retR = device_handler_->edg_servo_j(1u, &jr, MoveMode::ABS);
  if (retL != ERR_SUCC || retR != ERR_SUCC) {
    DEV_FATAL("Write positions failed!");
    return hardware_interface::return_type::ERROR;
  }
  int send_ret = device_handler_->edg_send();
  if (send_ret != ERR_SUCC) {
    DEV_FATAL("edg_send failed %d", send_ret);
    return hardware_interface::return_type::ERROR;
  }
#endif
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn
jaka_dual_control::JakaDualHardware::read_configure(
    const hardware_interface::HardwareInfo &info) {
  DEV_INFO("Reading configure ...please wait...");
  try {
    auto it_ip_addr = info.hardware_parameters.find("ip_addr");
    if (it_ip_addr == info.hardware_parameters.end()) {
      DEV_FATAL("port not found or invalid");
      return hardware_interface::CallbackReturn::ERROR;
    }
    ip_addr_ = it_ip_addr->second;
    auto it_rpy = info.hardware_parameters.find("rpy");
    if (it_rpy == info.hardware_parameters.end()) {
      DEV_FATAL("rpy not found or invalid");
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::string rpy_str = it_rpy->second;
    DEV_INFO("RPY: %s", rpy_str.c_str());
    std::istringstream iss(rpy_str);
    for (int i = 0; i < 3; ++i) {
      if (!(iss >> rpy_[i])) {
        DEV_FATAL("rpy format invalid");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  } catch (const std::exception &e) {
    DEV_FATAL("slave_id not found or invalid");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
jaka_dual_control::JakaDualHardware::OpenDevice() {
  DEV_INFO("Opening device ...please wait...");
  DEV_INFO("IP Adress: %s", ip_addr_.c_str());
  DEV_INFO("RPY: [%f, %f, %f]", rpy_[0], rpy_[1], rpy_[2]);
  device_handler_ = std::make_unique<JAKAZuRobot>();
  errno_t res = device_handler_->login_in(ip_addr_.c_str());
  if (res != ERR_SUCC) {
    DEV_FATAL("Login in failed! Error code: %d", res);
    return hardware_interface::CallbackReturn::ERROR;
  }
  DEV_INFO("************************Login in success!");
  res = device_handler_->power_on();
  if (res != ERR_SUCC) {
    DEV_FATAL("Power on failed! Error code: %d", res);
    return hardware_interface::CallbackReturn::ERROR;
  }
  DEV_INFO("************************Power on success!");
  res = device_handler_->enable_robot();
  if (res != ERR_SUCC) {
    DEV_FATAL("Enable robot failed! Error code: %d", res);
    return hardware_interface::CallbackReturn::ERROR;
  }
  DEV_INFO("************************Enable robot success!");
#if 1
  res = device_handler_->servo_move_enable(TRUE, -1);
  if (res != ERR_SUCC) {
    DEV_FATAL("Enable servo move mode failed! Error code: %d", res);
    return hardware_interface::CallbackReturn::ERROR;
  }
  DEV_INFO("************************Servo Mode Change success!");
#endif
  res = device_handler_->set_gravity_direction(rpy_);
  if (res != ERR_SUCC) {
    DEV_FATAL("Set gravity direction failed! Error code: %d", res);
    return hardware_interface::CallbackReturn::ERROR;
  }
  DEV_INFO("Successfully opened!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace jaka_dual_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jaka_dual_control::JakaDualHardware,
                       hardware_interface::SystemInterface)
