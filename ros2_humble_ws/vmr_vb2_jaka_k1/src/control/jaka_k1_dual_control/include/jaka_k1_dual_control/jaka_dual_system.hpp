#ifndef JAKA_K1_DUAL_CONTROL_HPP_
#define JAKA_K1_DUAL_CONTROL_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "jaka_sdk/JAKAZuRobot.h"
#include "jaka_sdk/jkerr.h"
#include "jaka_sdk/jktypes.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

namespace jaka_dual_control {
class JakaDualHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JakaDualHardware);
  JAKA_DUAL_CONTROL_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  JAKA_DUAL_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  JAKA_DUAL_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  JAKA_DUAL_CONTROL_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  JAKA_DUAL_CONTROL_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  JAKA_DUAL_CONTROL_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  JAKA_DUAL_CONTROL_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  hardware_interface::CallbackReturn
  read_configure(const hardware_interface::HardwareInfo &info);

private:
  hardware_interface::CallbackReturn OpenDevice();
  hardware_interface::CallbackReturn CloseDevice();

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;

  std::string ip_addr_ = "192.168.2.200";
  std::unique_ptr<JAKAZuRobot> device_handler_ = nullptr;
  std::atomic_bool is_in_drag = false;
  std::atomic_bool is_in_collision = false;

  double rpy_[3]{0.0, 0.0, 0.0};
};

} // namespace jaka_dual_control

#endif // JAKA_K1_DUAL_CONTROL_HPP_
