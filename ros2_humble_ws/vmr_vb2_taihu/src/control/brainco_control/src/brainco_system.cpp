/**
 *****************************Copyright (c) 2023 ZJU****************************
 * @file      : diff_test_system.cpp
 * @brief     : 差速小车硬件接口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-07-18       Hao Lion        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *
 *
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023 ZJU****************************
 */

 #include "brainco_control/brainco_system.hpp"

 #include <chrono>
 #include <cmath>
 #include <cstddef>
 #include <cstdint>
 #include <iostream>
 #include <limits>
 #include <memory>
 #include <mutex>
 #include <sstream>
 #include <stdexcept>
 #include <thread>
 #include <vector>
 
 #include "hardware_interface/types/hardware_interface_type_values.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "ros2_stark_controller/stark-sdk.h"
 #define DEG2RAD 0.017453292519943295769236907684886
 #define RAD2DEG 57.295779513082320876798154814105
 
 namespace {
 constexpr auto DEFAULT_COMMAND_TOPIC = "diff_test_cmd";
 constexpr auto DEFAULT_STATE_TOPIC = "diff_test_cmd";
 
 } // namespace
 #define DEV_INFO(...)                                                          \
   RCLCPP_INFO(rclcpp::get_logger("BraincoRevo1Hardware"), __VA_ARGS__)
 
 #define DEV_FATAL(...)                                                         \
                                                                                \
   RCLCPP_INFO(rclcpp::get_logger("BraincoRevo1Hardware"), __VA_ARGS__)
 
 namespace brainco_control {
 hardware_interface::CallbackReturn
 BraincoRevo1Hardware::on_init(const hardware_interface::HardwareInfo &info) {
   if (hardware_interface::SystemInterface::on_init(info) !=
       hardware_interface::CallbackReturn::SUCCESS) {
     return hardware_interface::CallbackReturn::ERROR;
   }
   positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   cmds_ = {10000, 10000, 10000, 10000, 10000, 10000};
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
 
   if (info_.joints.size() != 6) {
     RCLCPP_FATAL(rclcpp::get_logger("BraincoRevo1Hardware"),
                  "The number of crawlers is set not correactly,please check "
                  "your urdf. 2 is expected,but now it is %zu.",
                  info_.joints.size());
     return hardware_interface::CallbackReturn::ERROR;
   } else {
     RCLCPP_INFO(rclcpp::get_logger("BraincoRevo1Hardware"),
                 "Found 2 crawlers sucessfully!");
   }
 
   hw_positions_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
   hw_commands_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
 
   for (const hardware_interface::ComponentInfo &joint : info_.joints) {
     // DiffBotSystem has exactly two states and one command interface on each
     // joint
     if (joint.command_interfaces.size() != 1) {
       RCLCPP_FATAL(rclcpp::get_logger("BraincoRevo1Hardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
       return hardware_interface::CallbackReturn::ERROR;
     }
 
     if (joint.command_interfaces[0].name !=
         hardware_interface::HW_IF_POSITION) {
       RCLCPP_FATAL(
           rclcpp::get_logger("BraincoRevo1Hardware"),
           "Joint '%s' have %s command interfaces found. '%s' expected.",
           joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
           hardware_interface::HW_IF_POSITION);
       return hardware_interface::CallbackReturn::ERROR;
     }
 
     if (joint.state_interfaces.size() != 1) {
       RCLCPP_FATAL(rclcpp::get_logger("BraincoRevo1Hardware"),
                    "Joint '%s' has %zu state interface. 1 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
       return hardware_interface::CallbackReturn::ERROR;
     }
 
     if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
       RCLCPP_FATAL(
           rclcpp::get_logger("BraincoRevo1Hardware"),
           "Joint '%s' have '%s' as first state interface. '%s' expected.",
           joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
           hardware_interface::HW_IF_POSITION);
       return hardware_interface::CallbackReturn::ERROR;
     }
   }
 
   return hardware_interface::CallbackReturn::SUCCESS;
 }
 
 std::vector<hardware_interface::StateInterface>
 BraincoRevo1Hardware::export_state_interfaces() {
   std::vector<hardware_interface::StateInterface> state_interfaces;
   for (auto i = 0u; i < info_.joints.size(); i++) {
     state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_POSITION,
         &hw_positions_[i]));
   }
 
   return state_interfaces;
 }
 
 std::vector<hardware_interface::CommandInterface>
 BraincoRevo1Hardware::export_command_interfaces() {
   std::vector<hardware_interface::CommandInterface> command_interfaces;
   for (auto i = 0u; i < info_.joints.size(); i++) {
     command_interfaces.emplace_back(hardware_interface::CommandInterface(
         info_.joints[i].name, hardware_interface::HW_IF_POSITION,
         &hw_commands_[i]));
   }
 
   return command_interfaces;
 }
 
 hardware_interface::CallbackReturn BraincoRevo1Hardware::on_activate(
     const rclcpp_lifecycle::State & /*previous_state*/) {
   // BEGIN: This part here is for exemplary purposes - Please do not copy to
   // your production code
   RCLCPP_INFO(rclcpp::get_logger("BraincoRevo1Hardware"),
               "Activating ...please wait...");
   auto motor_status = stark_get_motor_status(device_handler_, slave_id_);
   if (motor_status == nullptr) {
     DEV_FATAL("Get motor status failed!");
     return hardware_interface::CallbackReturn::ERROR;
   }
   for (int i = 0; i < 6; i++) {
      uint16_t final_num = 0;
      switch (i) {
      case 0:
        final_num = current_config_.max_thumb_aux_deg * (motor_status->positions[i] / current_config_.division);
        break;
      case 1:
        final_num = current_config_.max_thumb_flex_deg * (motor_status->positions[i] / current_config_.division);
        break;
      default:
        final_num = current_config_.other_finger_flex_deg * (motor_status->positions[i] / current_config_.division);
        break;
      }
      hw_positions_[i] = final_num * DEG2RAD;
      hw_commands_[i] = hw_positions_[i];
    }
 
   RCLCPP_INFO(rclcpp::get_logger("BraincoRevo1Hardware"),
               "Successfully activated!");
 
   return hardware_interface::CallbackReturn::SUCCESS;
 }
 
 hardware_interface::CallbackReturn BraincoRevo1Hardware::on_deactivate(
     const rclcpp_lifecycle::State & /*previous_state*/) {
   // BEGIN: This part here is for exemplary purposes - Please do not copy to
   // your production code
   RCLCPP_INFO(rclcpp::get_logger("BraincoRevo1Hardware"),
               "Deactivating ...please wait...");
   // END: This part here is for exemplary purposes - Please do not copy to your
   // production code
 
   RCLCPP_INFO(rclcpp::get_logger("BraincoRevo1Hardware"),
               "Successfully deactivated!");
 
   return hardware_interface::CallbackReturn::SUCCESS;
 }
 
 hardware_interface::return_type
 brainco_control::BraincoRevo1Hardware::read(const rclcpp::Time & /*time*/,
                                             const rclcpp::Duration &period) {
 #if 0
   auto motor_status = stark_get_motor_status(device_handler_, slave_id_);
   if (motor_status == nullptr) {
     DEV_FATAL("Get motor status failed!");
     return hardware_interface::return_type::ERROR;
   }
 nt i = 0; i < 6; i++) {
     uint16_t final_num = 0;
     switch (i) {
     case 0:
       final_num = 90 * (motor_status->positions[i] / 100.0);
       break;
     case 1:
       final_num = 55 * (motor_status->positions[i] / 100.0);
       break;
     default:
       final_num = 70 * (motor_status->positions[i] / 100.0);
       break;
     }
     hw_positions_[i] = final_num * DEG2RAD;
   }
 #endif
 #if 1
   // DEV_INFO("Slave_id:%d ,Read positions_: %f, %f, %f, %f, %f, %f", slave_id_,
   //          this->tmp_positions_[0], this->tmp_positions_[1],
   //          this->tmp_positions_[2], this->tmp_positions_[3],
   //          this->tmp_positions_[4], this->tmp_positions_[5]);
   double positions[6];
   {
     std::lock_guard<std::mutex> lock(positions_mtx);
     for (int i = 0; i < 6; ++i) {
       positions[i] = tmp_positions_[i];
     }
     // DEV_INFO("Slave_id:%d ,Read positions: %f, %f, %f, %f, %f, %f", slave_id_,
     //          positions[0], positions[1], positions[2], positions[3],
     //          positions[4], positions[5]);
     // DEV_INFO("Slave_id:%d ,Read positions_: %f, %f, %f, %f, %f, %f",
     // slave_id_,
     //          this->positions_[0], this->positions_[1], this->positions_[2],
     //          this->positions_[3], this->positions_[4], this->positions_[5]);
   }
   for (int i = 0; i < 6; ++i) {
     // DEV_INFO("Read position[%d]: %u", i, positions[i]);
     hw_positions_[i] = positions[i];
   }
   // DEV_INFO("Read positions: %f, %f, %f, %f, %f, %f", hw_positions_[0], hw_positions_[1],
   //   hw_positions_[2], hw_positions_[3], hw_positions_[4], hw_positions_[5]);
 #endif
   return hardware_interface::return_type::OK;
 }
 
 hardware_interface::return_type brainco_control::BraincoRevo1Hardware::write(
     const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
 #if 0
   uint16_t positions[6];
   for (int i = 0; i < 6; i++) {
     double final_num = 0;
     switch (i) {
     case 0:
       final_num = hw_commands_[i] * RAD2DEG / 90 * 100;
       break;
     case 1:
       final_num = hw_commands_[i] * RAD2DEG / 55 * 100;
       break;
     default:
       final_num = hw_commands_[i] * RAD2DEG / 70 * 100;
       break;
     }
     if (final_num > 100) {
       final_num = 100;
     }
     positions[i] = static_cast<uint16_t>(final_num);
     //positions[i] = static_cast<uint16_t>(hw_commands_[i] * RAD2DEG);
   }
   // DEV_INFO("Set positions: %s", oss.str().c_str());
   // DEV_INFO("Set positions Unc: %s", oss1.str().c_str());
   stark_set_finger_positions(device_handler_, slave_id_, positions, 6);
 #endif
 #if 1
   uint16_t cmds[6];
   for (int i = 0; i < 6; ++i) {
     double final_num = 0;
     switch (i) {
     case 0:
       final_num = hw_commands_[i] * RAD2DEG / current_config_.max_thumb_aux_deg * current_config_.division;
       break;
     case 1:
       final_num = hw_commands_[i] * RAD2DEG / current_config_.max_thumb_flex_deg * current_config_.division;
       break;
     default:
       final_num = hw_commands_[i] * RAD2DEG / current_config_.other_finger_flex_deg * current_config_.division;
       break;
     }
     if (final_num > current_config_.division) {
       final_num = current_config_.division;
     }
     cmds[i] = static_cast<uint16_t>(final_num);
   }
   {
     std::lock_guard<std::mutex> lock(commands_mtx);
     for (int i = 0; i < 6; ++i) {
       cmds_[i] = cmds[i];
     }
   }
 #endif
   return hardware_interface::return_type::OK;
 }
 
 hardware_interface::CallbackReturn
 brainco_control::BraincoRevo1Hardware::read_configure(
     const hardware_interface::HardwareInfo &info) {
   DEV_INFO("Reading configure ...please wait...");
   try {
     auto it_slave_id = info.hardware_parameters.find("slave_id");
     if (it_slave_id == info.hardware_parameters.end()) {
       DEV_FATAL("slave_id not found or invalid");
       return hardware_interface::CallbackReturn::ERROR;
     }
     slave_id_ = std::stoi(it_slave_id->second);
     DEV_INFO("slave_id: %d", slave_id_);
     auto it_port = info.hardware_parameters.find("port");
     if (it_port == info.hardware_parameters.end()) {
       DEV_FATAL("port not found or invalid");
       return hardware_interface::CallbackReturn::ERROR;
     }
     port_ = it_port->second;
     DEV_INFO("port: %s", port_.c_str());
     auto it_baudrate = info.hardware_parameters.find("baudrate");
     if (it_baudrate == info.hardware_parameters.end()) {
       DEV_FATAL("baudrate not found or invalid");
       return hardware_interface::CallbackReturn::ERROR;
     }
     baudrate_ = std::stoi(it_baudrate->second);
     DEV_INFO("baudrate: %d", baudrate_);
     auto it_version = info.hardware_parameters.find("version");
     if (it_version == info.hardware_parameters.end()) {
       DEV_FATAL("version not found or invalid");
       return hardware_interface::CallbackReturn::ERROR;
     }
     DEV_INFO("version: %s", it_version->second.c_str());
     if (it_version->second == "revo1") {
       current_config_ = revo1_config_;
     } else if (it_version->second == "revo2") {
       current_config_ = revo2_config_;
     } else {
       DEV_FATAL("version invalid");
       return hardware_interface::CallbackReturn::ERROR;
     }
   } catch (const std::exception &e) {
     DEV_FATAL("slave_id not found or invalid");
     return hardware_interface::CallbackReturn::ERROR;
   }
   return hardware_interface::CallbackReturn::SUCCESS;
 }
 
 hardware_interface::CallbackReturn
 brainco_control::BraincoRevo1Hardware::OpenDevice() {
   DEV_INFO("Opening device ...please wait...");
   DEV_INFO("port: %s, baudrate: %d, slave_id: %d", port_.c_str(), baudrate_,
            slave_id_);
   device_handler_ = modbus_open(port_.c_str(), baudrate_);
   if (device_handler_ == nullptr) {
     DEV_FATAL("Open device failed!");
     return hardware_interface::CallbackReturn::ERROR;
   }
   auto info = stark_get_device_info(device_handler_, slave_id_);
   if (info == nullptr) {
     DEV_FATAL("Get device info failed!");
     return hardware_interface::CallbackReturn::ERROR;
   }
   DEV_INFO("Device info: Slave[%d] SKU Type: %hhu, Serial Number: %s, "
            "Firmware Version: %s\n",
            slave_id_, (uint8_t)info->sku_type, info->serial_number,
            info->firmware_version);
   DEV_INFO("Successfully opened!");
 #if 1
   read_thread_ = std::thread([this]() {
     rclcpp::Rate rate(50);
     while (1) {
       auto motor_status = stark_get_motor_status(device_handler_, slave_id_);
       if (motor_status == nullptr) {
         DEV_FATAL("Get motor status failed!");
         return hardware_interface::return_type::ERROR;
       }
       double positions[6];
       std::ostringstream oss;
       for (int i = 0; i < 6; i++) {
         // oss << "ori:" <<motor_status->positions[i] << " ";
         uint16_t final_num = 0;
         switch (i) {
         case 0:
           final_num = current_config_.max_thumb_aux_deg * (motor_status->positions[i] / current_config_.division);
           break;
         case 1:
           final_num = current_config_.max_thumb_flex_deg * (motor_status->positions[i] / current_config_.division);
           break;
         default:
           final_num = current_config_.other_finger_flex_deg * (motor_status->positions[i] / current_config_.division);
           break;
         }
         positions[i] = final_num * DEG2RAD;
         // oss << "final:" <<positions[i] << ", ";
       }
       // DEV_INFO(oss.str().c_str());
       {
         std::lock_guard<std::mutex> lock(positions_mtx);
         // DEV_INFO("Slave_id:%d ,Read positions: %f, %f, %f, %f, %f, %f",
         // slave_id_,
         //          positions[0], positions[1], positions[2], positions[3],
         //          positions[4], positions[5]);
         for (int i = 0; i < 6; ++i) {
           tmp_positions_[i] = positions[i];
         }
         // DEV_INFO("Slave_id:%d ,111Read positions_: %f, %f, %f, %f, %f, %f",
         //          slave_id_, tmp_positions_[0], tmp_positions_[1],
         //          tmp_positions_[2], tmp_positions_[3], tmp_positions_[4],
         //          tmp_positions_[5]);
       }
       rate.sleep();
     }
   });
 #endif
 #if 1
   write_thread_ = std::thread([this]() {
     rclcpp::Rate rate(50);
     while (1) {
       uint16_t cmds[6]{10000, 10000, 10000, 10000, 10000, 10000};
       {
         std::lock_guard<std::mutex> lock(commands_mtx);
         for (int i = 0; i < 6; ++i) {
           cmds[i] = cmds_[i];
         }
       }
       std::ostringstream oss;
       if (cmds[0] < 10000) {
         // for (int i = 0; i < 6; ++i) {
         //  oss << cmds[i] << " ";
         // }
         stark_set_finger_positions(device_handler_, slave_id_, cmds, 6);
       }
       // DEV_INFO("slave_id: %d, set cmds: %s", slave_id_, oss.str().c_str());
       rate.sleep();
     }
   });
 #endif
   DEV_INFO("Device opened successfully!");
   return hardware_interface::CallbackReturn::SUCCESS;
 }
 
 } // namespace brainco_control
 
 #include "pluginlib/class_list_macros.hpp"
 PLUGINLIB_EXPORT_CLASS(brainco_control::BraincoRevo1Hardware,
                        hardware_interface::SystemInterface)
 