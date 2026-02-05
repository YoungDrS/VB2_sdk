#include "taihu_dual_arm_hardware/taihu_dual_arm_hardware_interface.hpp"
#include <vector>

namespace taihu_dual_arm_hardware{

hardware_interface::CallbackReturn TaihuDualArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = sysinfo;//info_是父类中定义的变量

    _joint_num = info_.joints.size();
    
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }


        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("TaihuDualArmHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }


    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> TaihuDualArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 机械臂1的关节状态接口(关节0-7)
  for (size_t i = 0; i < 7; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, 
      hardware_interface::HW_IF_POSITION, 
      &_jnt_position_state1[i]));
  }

  // 机械臂2的关节状态接口(关节7-14)
  for (size_t i = 7; i < 14; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, 
      hardware_interface::HW_IF_POSITION, 
      &_jnt_position_state2[i-7]));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TaihuDualArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // 机械臂1的关节命令接口(关节0-7)
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, 
      hardware_interface::HW_IF_POSITION, 
      &_jnt_position_command1[i]));
  }

  // 机械臂2的关节命令接口(关节7-14)
  for (size_t i = 7; i < 14; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, 
      hardware_interface::HW_IF_POSITION, 
      &_jnt_position_command2[i-7]));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn TaihuDualArmHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Starting ...please wait...");

    uint8_t cmd_v_p[7] = {36, 36, 36, 36, 36, 36, 36};   // 最大速度；正向速度
    uint8_t cmd_v_n[7] = {37, 37, 37, 37, 37, 37, 37};   // 最小速递；负向速度
    uint32_t para_v_p[7] = {5559, 5559, 6043, 6043, 6542, 7354, 7354};
    uint32_t para_v_n[7] = {(uint32_t)-5559, (uint32_t)-5559, (uint32_t)-6043, (uint32_t)-6043, (uint32_t)-6542, (uint32_t)-7354, (uint32_t)-7354};

    // 初始化 cncalyst
    if (!init_canalyst()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒
    sendCanCommandWrite(7, _can_id_list_1, cmd_v_p, para_v_p, _n_CAN_ind_1); // 设置正向速度
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒
    sendCanCommandWrite(7, _can_id_list_1, cmd_v_n, para_v_n, _n_CAN_ind_1); // 设置负向速度
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒
    sendCanCommandWrite(7, _can_id_list_2, cmd_v_p, para_v_p, _n_CAN_ind_2); // 设置正向速度
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒
    sendCanCommandWrite(7, _can_id_list_2, cmd_v_n, para_v_n, _n_CAN_ind_2); // 设置负向速度

    // 初始化变量
    for(int i=0; i<7; i++){
        _jnt_position_command1[i] = 0;
        _jnt_position_state1[i] = 0;

        _jnt_position_command2[i] = 0;
        _jnt_position_state2[i] = 0;
    }
    // 获取机械臂初始状态
    std::vector<int> read_data_1;
    std::vector<int> read_data_2;
    std::vector<uint8_t> command_list(7, 8); 
    read_data_1 = sendCanCommandRead(7, _can_id_list_1, command_list.data(), _n_CAN_ind_1);
    read_data_2 = sendCanCommandRead(7, _can_id_list_2, command_list.data(), _n_CAN_ind_2);

    // 机械臂1的关节状态读取(关节0-7)
    for (size_t i = 0; i < 7; ++i) {
        _jnt_position_command1[i] = read_data_1[i] / 728.18  * 3.14159 / 180;
    }
    // 机械臂2的关节状态读取(关节7-14)
    for (size_t i = 0; i < 7; ++i) {
        _jnt_position_command2[i] = read_data_2[i] / 728.18  * 3.14159 / 180;
    }


    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Initial command positions for Robot 1: %f,%f,%f,%f,%f,%f,%f", 
        _jnt_position_command1[0], _jnt_position_command1[1], _jnt_position_command1[2], 
        _jnt_position_command1[3], _jnt_position_command1[4], _jnt_position_command1[5], _jnt_position_command1[6]);
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Initial command positions for Robot 2: %f,%f,%f,%f,%f,%f,%f", 
        _jnt_position_command2[0], _jnt_position_command2[1], _jnt_position_command2[2], 
        _jnt_position_command2[3], _jnt_position_command2[4], _jnt_position_command2[5], _jnt_position_command2[6]);
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Both robots successfully started!");


    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn TaihuDualArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Stopping ...please wait...");

    VCI_ClearBuffer(_n_device_type, _n_device_ind, _n_CAN_ind_1); // 清空CAN缓冲区
    VCI_ClearBuffer(_n_device_type, _n_device_ind, _n_CAN_ind_2); // 清空CAN缓冲区
    VCI_CloseDevice(_n_device_type,_n_device_ind); // 关闭CAN设备
    
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type TaihuDualArmHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    std::vector<int> read_data_1;
    std::vector<int> read_data_2;
    std::vector<uint8_t> command_list(7, 8); 
    read_data_1 = sendCanCommandRead(7, _can_id_list_1, command_list.data(), _n_CAN_ind_1);
    read_data_2 = sendCanCommandRead(7, _can_id_list_2, command_list.data(), _n_CAN_ind_2);

    // 机械臂1的关节状态读取(关节0-7)
    for (size_t i = 0; i < 7; ++i) {
        _jnt_position_state1[i] = read_data_1[i] / 728.18  * 3.14159 / 180;
    }

    // 机械臂2的关节状态读取(关节7-14)
    for (size_t i = 0; i < 7; ++i) {
        _jnt_position_state2[i] = read_data_2[i] / 728.18  * 3.14159 / 180;
    }

    // RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Initial command positions for Robot 1: %f,%f,%f,%f,%f,%f,%f", 
    //     _jnt_position_state1[0], _jnt_position_state1[1], _jnt_position_state1[2], 
    //     _jnt_position_state1[3], _jnt_position_state1[4], _jnt_position_state1[5],_jnt_position_state1[6]);
    // RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Initial command positions for Robot 2: %f,%f,%f,%f,%f,%f,%f", 
    //     _jnt_position_state2[0], _jnt_position_state2[1], _jnt_position_state2[2], 
    //     _jnt_position_state2[3], _jnt_position_state2[4], _jnt_position_state2[5],_jnt_position_state2[6]);


    return hardware_interface::return_type::OK;

}

hardware_interface::return_type TaihuDualArmHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    std::vector<double> now_position_1(7, 0.0);
    std::vector<double> now_position_2(7, 0.0);
    std::vector<uint32_t> target_position_1(7, 0); 
    std::vector<uint32_t> target_position_2(7, 0); 
    std::vector<uint8_t> command_list_1(7, 30); //CAN设置位置的指令
    std::vector<uint8_t> command_list_2(7, 30); //CAN设置位置的指令


    // 左臂
    if (std::any_of(&_jnt_position_command1[0], &_jnt_position_command1[6],\
        [](double c) { return not std::isfinite(c); })) {
        return hardware_interface::return_type::ERROR;
    }
    for (size_t joint_id = 0; joint_id < 7; ++joint_id)
    {

        now_position_1[joint_id] = _jnt_position_command1[joint_id] * 728.18 * 180 / 3.14159; //将弧度转换为编码器数值
        
    }
	target_position_1[0] = static_cast<uint32_t>(int(now_position_1[0])); //编码器数值表示的角度值
	target_position_1[1] = static_cast<uint32_t>(int(now_position_1[1])); 
	target_position_1[2] = static_cast<uint32_t>(int(now_position_1[2]));
	target_position_1[3] = static_cast<uint32_t>(int(now_position_1[3]));
	target_position_1[4] = static_cast<uint32_t>(int(now_position_1[4]));
	target_position_1[5] = static_cast<uint32_t>(int(now_position_1[5]));
    target_position_1[6] = static_cast<uint32_t>(int(now_position_1[6]));

    // 右臂
    if (std::any_of(&_jnt_position_command2[0], &_jnt_position_command2[7],\
        [](double c) { return not std::isfinite(c); })) {
        return hardware_interface::return_type::ERROR;
    }
    for (size_t joint_id = 0; joint_id < 7; ++joint_id)
    {
      now_position_2[joint_id] = _jnt_position_command2[joint_id] * 728.18 * 180 / 3.14159;
    }
	target_position_2[0] = static_cast<uint32_t>(int(now_position_2[0]));
	target_position_2[1] = static_cast<uint32_t>(int(now_position_2[1]));
	target_position_2[2] = static_cast<uint32_t>(int(now_position_2[2]));
	target_position_2[3] = static_cast<uint32_t>(int(now_position_2[3]));
	target_position_2[4] = static_cast<uint32_t>(int(now_position_2[4]));
  	target_position_2[5] = static_cast<uint32_t>(int(now_position_2[5]));
  	target_position_2[6] = static_cast<uint32_t>(int(now_position_2[6]));


    // 发送 CAN 命令
    if (!sendCanCommandWrite(7, _can_id_list_1, command_list_1.data(), target_position_1.data(), _n_CAN_ind_1)) {
        RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "发送 CAN通道1 命令失败");
        return hardware_interface::return_type::ERROR;
    }
    if (!sendCanCommandWrite(7, _can_id_list_2, command_list_2.data(), target_position_2.data(), _n_CAN_ind_2)) {
        RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "发送 CAN通道2 命令失败");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


// 初始化 canalyst
bool TaihuDualArmHardwareInterface::init_canalyst()
{
    //打开设备：注意一个设备只能打开一次
	if(VCI_OpenDevice(_n_device_type,_n_device_ind,0)==1)
	{
        RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Canalyst 打开设备成功");     
	}else
	{
        RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "Canalyst 打开设备失败，请检查设备连接");
        return false; 
	}

	//配置CAN
	VCI_INIT_CONFIG config; 
	config.AccCode = 0x80000000;
	config.AccMask = 0xFFFFFFFF;
	config.Filter = 2;              //只接收标准帧
	config.Timing0 = 0x00; 
	config.Timing1 = 0x14;          //波特率定为1000kps  0x00 0x14
	config.Mode = 0;                //正常模式	   

    VCI_ResetCAN(_n_device_type, _n_device_ind, _n_CAN_ind_1);
    if (VCI_InitCAN(_n_device_type, _n_device_ind, _n_CAN_ind_1, &config) != 1)
    {
        VCI_CloseDevice(_n_device_type, _n_device_ind);
        RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "初始化 CAN通道1 失败，请检查设备连接");
        return false;
    }
    VCI_ClearBuffer(_n_device_type, _n_device_ind, _n_CAN_ind_1);
    if (VCI_StartCAN(_n_device_type, _n_device_ind, _n_CAN_ind_1) != 1)
    {
        VCI_CloseDevice(_n_device_type, _n_device_ind);
        RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "启动 CAN通道1 失败，请检查设备连接");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "启动 CAN通道1 成功");  


    VCI_ResetCAN(_n_device_type, _n_device_ind, _n_CAN_ind_2);
    if (VCI_InitCAN(_n_device_type, _n_device_ind, _n_CAN_ind_2, &config) != 1)
    {
        VCI_CloseDevice(_n_device_type, _n_device_ind);
        RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "初始化 CAN通道2 失败，请检查设备连接");
        return false;
    }
    VCI_ClearBuffer(_n_device_type, _n_device_ind, _n_CAN_ind_2);
    if (VCI_StartCAN(_n_device_type, _n_device_ind, _n_CAN_ind_2) != 1)
    {
        VCI_CloseDevice(_n_device_type, _n_device_ind);
        RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "启动 CAN通道2 失败，请检查设备连接");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "启动 CAN通道2 成功");  
    
    return true;
	
}

// 发送 CAN 命令，写入关节数据；num_是关节数量，can_id_list_是CAN ID列表，command_list_是指令列表，parameter_list_是参数列表
bool TaihuDualArmHardwareInterface::sendCanCommandWrite(uint8_t num_, uint8_t* can_id_list_,
                                                uint8_t* command_list_, uint32_t* parameter_list_, uint8_t n_CAN_ind)
{

    // 定义并初始化 CAN 发送帧结构体
    VCI_CAN_OBJ send_frame = {};
    send_frame.SendType = 0;  //发送帧类型：0为正常发送
    send_frame.RemoteFlag = 0;  //0为数据帧，1为远程帧
    send_frame.ExternFlag = 0;  //0为标准帧，1为拓展帧
    send_frame.DataLen = 5;

    for (int i = 0; i < num_; ++i)
    {
        send_frame.ID       = can_id_list_[i];       // 设置目标CAN设备ID
        send_frame.Data[0]  = command_list_[i];     // 设置指令码

        // 将 uint32 参数转换为小端字节序，填入 Data[1~4]
        unsigned int value = static_cast<unsigned int>(parameter_list_[i]);
        for (int j = 1; j < 5; ++j)
        {
            send_frame.Data[j] = value & 0xFF;  // 取最低8位
            value >>= 8;  // 右移8位处理下一字节
        }

        // 发送数据帧
        if (VCI_Transmit(_n_device_type, _n_device_ind, n_CAN_ind, &send_frame, 1) != 1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "发送失败，CAN ID: %d", static_cast<int>(send_frame.ID));
            return false; 
        }
    }

    return true;
}

// 发送 CAN 命令，读取关节数据；num_是关节数量，can_id_list_是CAN ID列表，command_list_是指令列表
std::vector<int> TaihuDualArmHardwareInterface::sendCanCommandRead(uint8_t num_, uint8_t* can_id_list_,
                                                           uint8_t* command_list_, uint8_t n_CAN_ind)
{
    std::vector<int> result;  // 用于保存每个 actuator 的返回值（解码后）

    VCI_CAN_OBJ send_frame = {};
    send_frame.SendType   = 0;  
    send_frame.RemoteFlag = 0;  
    send_frame.ExternFlag = 0; 
    send_frame.DataLen    = 1;  

    for (int i = 0; i < num_; ++i)
    {
        send_frame.ID      = can_id_list_[i];     
        send_frame.Data[0] = command_list_[i];   

        // 发送命令帧
        if (VCI_Transmit(_n_device_type, _n_device_ind, n_CAN_ind, &send_frame, 1) != 1)
        {
            VCI_ClearBuffer(_n_device_type, _n_device_ind, n_CAN_ind); // 清空CAN缓冲区
            RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "发送失败，CAN ID: %d", static_cast<int>(send_frame.ID));
            break;  // 发送失败则终止后续操作
        }

        // 接收回应：最多尝试5次，每次等待100ms
        const int max_tries = 5;
        int try_count = max_tries;
        int rec_len = 0;
        VCI_CAN_OBJ recv_frames[2500];

        while ((rec_len = VCI_Receive(_n_device_type, _n_device_ind, n_CAN_ind, recv_frames, 2500, 100)) <= 0 && try_count > 0)
        {
            --try_count;
        }

        if (try_count == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TaihuDualArmHardwareInterface"), "CAN ID %d 接收超时，重试 %d 次失败", static_cast<int>(send_frame.ID), max_tries);
            // 读取时失败时，把当前关节的解码值设置为0；此处修改解决了一个断线后，后面关节乱动问题
            result.push_back(0); 
        }
        else
        {
            // 解析所有接收到的反馈帧
            for (int j = 0; j < rec_len; ++j)
            {
                const auto& frame = recv_frames[j];

                // 提取Data[1~4]组成4字节整数（小端 → 大端）
                uint32_t raw = 0;
                raw |= (static_cast<uint32_t>(frame.Data[1]) << 0);
                raw |= (static_cast<uint32_t>(frame.Data[2]) << 8);
                raw |= (static_cast<uint32_t>(frame.Data[3]) << 16);
                raw |= (static_cast<uint32_t>(frame.Data[4]) << 24);

                // 转换为带符号整数（补码表示）
                int32_t decoded = (raw > 0x7FFFFFFF) ? (raw - 0x100000000) : raw;

                result.push_back(decoded);  // 保存转换后的值
            }


        }
    }

    return result;
}





}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(taihu_dual_arm_hardware::TaihuDualArmHardwareInterface, hardware_interface::SystemInterface)
