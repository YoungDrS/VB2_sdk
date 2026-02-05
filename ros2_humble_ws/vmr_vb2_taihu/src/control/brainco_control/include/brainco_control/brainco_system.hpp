#ifndef BRAINCO_R1_CONTROL_HPP_
#define BRAINCO_R1_CONTROL_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros2_stark_controller/stark-sdk.h"
#include "ros2_stark_controller/zcan.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "visibility_control.h"

namespace brainco_control {
class BraincoRevo1Hardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BraincoRevo1Hardware);

  BRAINCO_REVO1_CONTROL_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  BRAINCO_REVO1_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  BRAINCO_REVO1_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  BRAINCO_REVO1_CONTROL_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  BRAINCO_REVO1_CONTROL_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  BRAINCO_REVO1_CONTROL_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  BRAINCO_REVO1_CONTROL_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  BRAINCO_REVO1_CONTROL_PUBLIC
  hardware_interface::CallbackReturn
  read_configure(const hardware_interface::HardwareInfo &info);

private:
  hardware_interface::CallbackReturn OpenDevice();
  hardware_interface::CallbackReturn CloseDevice();

  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::shared_ptr<rclcpp::Node> node_;
  // subscribe crawler feed back
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      fb_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64MultiArray>>
      received_fb_msg_ptr_{nullptr};
  // publishe crawler conmand
  std::shared_ptr<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      realtime_cmd_publisher_ = nullptr;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>
      cmd_publisher = nullptr;

  int slave_id_ = -1;
  std::string port_ = "/dev/ttyUSB0";
  int baudrate_ = 115200;
  DeviceHandler *device_handler_ = nullptr;
  std::thread read_thread_;
  std::mutex positions_mtx;
  std::vector<double> positions_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> tmp_positions_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

  std::thread write_thread_;
  std::mutex commands_mtx;
  std::vector<uint16_t> cmds_;

  struct HandPosConfig {
    double max_thumb_flex_deg = 60.0;
    double max_thumb_aux_deg = 87.0;
    double other_finger_flex_deg = 84.0;
    double division = 1000;
    HandPosConfig() = default;
    HandPosConfig(double _max_thumb_flex_deg, double _max_thumb_aux_deg, double _other_finger_flex_deg, int _division)
    : max_thumb_flex_deg(_max_thumb_flex_deg),
      max_thumb_aux_deg(_max_thumb_aux_deg),
      other_finger_flex_deg(_other_finger_flex_deg),
      division(_division) {}
    ~HandPosConfig() = default;
    HandPosConfig(const HandPosConfig &other)
    : max_thumb_flex_deg(other.max_thumb_flex_deg),
      max_thumb_aux_deg(other.max_thumb_aux_deg),
      other_finger_flex_deg(other.other_finger_flex_deg),
      division(other.division) {}
  };
  const HandPosConfig revo1_config_ = HandPosConfig(55.0, 90.0, 70.0, 100);
  const HandPosConfig revo2_config_ = HandPosConfig(60.0, 87.0, 84.0, 1000);
  HandPosConfig current_config_;
};

} // namespace brainco_control

#endif // BRAINCO_R1_CONTROL_HPP_
