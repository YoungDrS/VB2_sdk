#include "canalyst/controlcan.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("taihu_motor_disable");

  // 打开和初始化设备的一些参数配置
  int n_device_type = 4; // 设备类型：CANalyst-II就用4
  int n_device_ind = 0;  // 设备索引：1个USB-CAN适配器就是0
  int n_CAN_ind_1 = 0;   // CAN通道1
  int n_CAN_ind_2 = 1;   // CAN通道2

  uint8_t can_id_list_1[7] = {23, 24, 25, 26, 27, 28, 29}; //左臂
  uint8_t can_id_list_2[7] = {16, 17, 18, 19, 20, 21, 22}; //右臂

  // 打开设备：注意一个设备只能打开一次
  if (VCI_OpenDevice(n_device_type, n_device_ind, 0) == 1) {
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                "Canalyst 打开设备成功");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                 "Canalyst 打开设备失败，请检查设备连接");
    return 0;
  }

  // 配置CAN
  VCI_INIT_CONFIG config;
  config.AccCode = 0x80000000;
  config.AccMask = 0xFFFFFFFF;
  config.Filter = 2; // 只接收标准帧
  config.Timing0 = 0x00;
  config.Timing1 = 0x14; // 波特率定为1000kps  0x00 0x14
  config.Mode = 0;       // 正常模式

  VCI_ResetCAN(n_device_type, n_device_ind, n_CAN_ind_1);
  if (VCI_InitCAN(n_device_type, n_device_ind, n_CAN_ind_1, &config) != 1) {
    VCI_CloseDevice(n_device_type, n_device_ind);
    RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                 "初始化 CAN通道1 成功失败，请检查设备连接");
    return 0;
  }
  VCI_ClearBuffer(n_device_type, n_device_ind, n_CAN_ind_1);
  if (VCI_StartCAN(n_device_type, n_device_ind, n_CAN_ind_1) != 1) {
    VCI_CloseDevice(n_device_type, n_device_ind);
    RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                 "启动 CAN通道1 成功失败，请检查设备连接");
    return 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
              "启动 CAN通道1 成功");

  VCI_ResetCAN(n_device_type, n_device_ind, n_CAN_ind_2);
  if (VCI_InitCAN(n_device_type, n_device_ind, n_CAN_ind_2, &config) != 1) {
    VCI_CloseDevice(n_device_type, n_device_ind);
    RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                 "初始化 CAN通道2 成功失败，请检查设备连接");
    return 0;
  }
  VCI_ClearBuffer(n_device_type, n_device_ind, n_CAN_ind_2);
  if (VCI_StartCAN(n_device_type, n_device_ind, n_CAN_ind_2) != 1) {
    VCI_CloseDevice(n_device_type, n_device_ind);
    RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                 "启动 CAN通道2 成功失败，请检查设备连接");
    return 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
              "启动 CAN通道2 成功");

  // 定义并初始化 CAN 发送帧结构体
  VCI_CAN_OBJ send_frame = {};
  send_frame.SendType = 0;   // 发送帧类型：0为正常发送
  send_frame.RemoteFlag = 0; // 0为数据帧，1为远程帧
  send_frame.ExternFlag = 0; // 0为标准帧，1为拓展帧
  send_frame.DataLen = 5;
  std::vector<uint8_t> command_list_1(7, 2); // CAN设置位置的指令，去使能
  std::vector<uint8_t> command_list_2(7, 2); // CAN设置位置的指令
  std::vector<uint32_t> parameter_list(7, 0); 

  for (int i = 0; i < 7; ++i) {
    send_frame.ID = can_id_list_1[i];       // 设置目标CAN设备ID
    send_frame.Data[0] = command_list_1[i]; // 设置指令码
    // 将 uint32 参数转换为小端字节序，填入 Data[1~4]
    unsigned int value = static_cast<unsigned int>(parameter_list[i]);
    for (int j = 1; j < 5; ++j) {
      send_frame.Data[j] = value & 0xFF; // 取最低8位
      value >>= 8;                       // 右移8位处理下一字节
    }
    // 发送数据帧
    if (VCI_Transmit(n_device_type, n_device_ind, n_CAN_ind_1, &send_frame,
                     1) != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                   "发送失败，CAN ID: %d", static_cast<int>(send_frame.ID));
    }
  }

  for (int i = 0; i < 7; ++i) {
    send_frame.ID = can_id_list_2[i];       // 设置目标CAN设备ID
    send_frame.Data[0] = command_list_2[i]; // 设置指令码
    // 将 uint32 参数转换为小端字节序，填入 Data[1~4]
    unsigned int value = static_cast<unsigned int>(parameter_list[i]);
    for (int j = 1; j < 5; ++j) {
      send_frame.Data[j] = value & 0xFF; // 取最低8位
      value >>= 8;                       // 右移8位处理下一字节
    }
    // 发送数据帧
    if (VCI_Transmit(n_device_type, n_device_ind, n_CAN_ind_2, &send_frame,
                     1) != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                   "发送失败，CAN ID: %d", static_cast<int>(send_frame.ID));
    }
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
