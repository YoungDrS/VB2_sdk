#ifndef _LANXIN_HARDWARE_INTERFACE_
#define _LANXIN_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "taihu_dual_arm_hardware/visibility_control.h"
#include <vector>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "canalyst/controlcan.h"


namespace taihu_dual_arm_hardware
{

class TaihuDualArmHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TaihuDualArmHardwareInterface)

  TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 初始化 canalyst
  bool init_canalyst();
  // 发送 CAN 命令，写入关节数据
  bool sendCanCommandWrite(uint8_t num_, uint8_t* can_id_list_,
                          uint8_t* command_list_, uint32_t* parameter_list_, uint8_t n_CAN_ind);
  // 发送 CAN 命令，读取关节数据
  std::vector<int> sendCanCommandRead(uint8_t num_, uint8_t* can_id_list_,
                                      uint8_t* command_list_, uint8_t n_CAN_ind);
  
private:

  //打开和初始化设备的一些参数配置
	int _n_device_type = 4;    //设备类型：CANalyst-II就用4
  int _n_device_ind = 0;     //设备索引：1个USB-CAN适配器就是0
  int _n_CAN_ind_1 = 0;       //CAN通道1
  int _n_CAN_ind_2 = 1;       //CAN通道2

  uint8_t _can_id_list_1[7] = {23, 24, 25, 26, 27, 28, 29}; //左臂
  uint8_t _can_id_list_2[7] = {16, 17, 18, 19, 20, 21, 22}; //右臂

  // 关节数量
  size_t _joint_num;

  // 左臂 1 ；右臂 2 ；腰部 3
  double _jnt_position_command1[7];
  double _jnt_position_state1[7];
  double _jnt_position_command2[7];
  double _jnt_position_state2[7];
  double _jnt_position_command3[2];
  double _jnt_position_state3[2];



  // 创建node节点
  rclcpp::Node::SharedPtr _node;
};

} //end namespace


#endif
