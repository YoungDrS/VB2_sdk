#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lx_motor_interfaces/msg/motor_status.hpp"

class MultiJointController : public rclcpp::Node {
public:
  MultiJointController() : Node("multi_joint_controller") {
    control_target_name_ = this->declare_parameter<std::string>("control_target", "waistd");
    target_positions_ = this->declare_parameter<std::vector<double>>("target_position", {0.0});
    threshold_ = this->declare_parameter<double>("threshold", 0.01);
    period_ = this->declare_parameter<double>("period", 0.02);
    step_deg_ = this->declare_parameter<double>("step_deg", 0.2);

    if (period_ <= 0.0) {
      period_ = 0.02;
      RCLCPP_WARN(this->get_logger(), "period <= 0, fallback to 0.02");
    }

    constexpr double kPi = 3.14159265358979323846;
    step_rad_ = step_deg_ * kPi / 180.0;

    control_target_ = parse_control_target(control_target_name_);
    select_topics_by_target();

    const std::size_t n = target_positions_.size();
    reached_flags_.assign(n, false);
    cmd_positions_.assign(n, 0.0);
    actual_positions_.assign(n, 0.0);
    start_positions_.assign(n, 0.0);
    startup_phase_.assign(n, true);

    control_pub_ = this->create_publisher<lx_motor_interfaces::msg::MotorStatus>(control_topic_, 10);

    if (use_step_mode_) {
      state_sub_ = this->create_subscription<lx_motor_interfaces::msg::MotorStatus>(
          state_topic_, 10, std::bind(&MultiJointController::state_callback, this, std::placeholders::_1));
    } else {
      initialized_ = true;
    }

    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_),
                                     std::bind(&MultiJointController::send_control_command, this));

    RCLCPP_INFO(this->get_logger(),
                "control_target=%s, control_topic=%s, state_topic=%s, step_mode=%s, "
                "target_position(rad)=%s, threshold(rad)=%.6f, period(s)=%.6f (~%.1f Hz), "
                "step=%.3f deg (%.6f rad)",
                control_target_name_.c_str(), control_topic_.c_str(),
                state_topic_.empty() ? "N/A" : state_topic_.c_str(),
                use_step_mode_ ? "true" : "false", vector_to_string(target_positions_).c_str(), threshold_, period_,
                1.0 / period_, step_deg_, step_rad_);

    if (use_step_mode_ && target_positions_.size() > 64) {
      RCLCPP_WARN(this->get_logger(), "target_position size (%zu) > 64, only first 64 will be used",
                  target_positions_.size());
    }
    if (!use_step_mode_ && target_positions_.size() > 1) {
      RCLCPP_WARN(this->get_logger(),
                  "raise mode only uses target_position[0], remaining values are ignored");
    }
  }

private:
  enum class ControlTarget {
    kWaistd,
    kRotateHead,
    kNodHead,
    kRaise,
    kUnknown,
  };

  static std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
  }

  static ControlTarget parse_control_target(const std::string &name) {
    const std::string value = to_lower(name);
    if (value == "waistd" || value == "waist") {
      return ControlTarget::kWaistd;
    }
    if (value == "rotatehead") {
      return ControlTarget::kRotateHead;
    }
    if (value == "nodhead") {
      return ControlTarget::kNodHead;
    }
    if (value == "raise") {
      return ControlTarget::kRaise;
    }
    return ControlTarget::kUnknown;
  }

  void select_topics_by_target() {
    switch (control_target_) {
      case ControlTarget::kWaistd:
        control_topic_ = "/vmr/waist_joint_control";
        state_topic_ = "/vmr/waist_joint_state";
        use_step_mode_ = true;
        break;
      case ControlTarget::kRotateHead:
        control_topic_ = "/vmr/head_joint_control_1";
        state_topic_ = "/vmr/head_joint_state_1";
        use_step_mode_ = true;
        break;
      case ControlTarget::kNodHead:
        control_topic_ = "/vmr/head_joint_control_2";
        state_topic_ = "/vmr/head_joint_state_2";
        use_step_mode_ = true;
        break;
      case ControlTarget::kRaise:
        control_topic_ = "/vmr/raise_control";
        state_topic_ = "/vmr/raise_state";
        use_step_mode_ = false;
        break;
      case ControlTarget::kUnknown:
      default:
        RCLCPP_WARN(this->get_logger(),
                    "unknown control_target=%s, fallback to waistd", control_target_name_.c_str());
        control_target_ = ControlTarget::kWaistd;
        control_target_name_ = "waistd";
        control_topic_ = "/vmr/waist_joint_control";
        state_topic_ = "/vmr/waist_joint_state";
        use_step_mode_ = true;
        break;
    }
  }

  static double clamp_step(double diff, double step) {
    if (diff > step) {
      return step;
    }
    if (diff < -step) {
      return -step;
    }
    return diff;
  }

  static std::string vector_to_string(const std::vector<double> &v) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
      oss << v[i];
      if (i + 1 < v.size()) {
        oss << ", ";
      }
    }
    oss << "]";
    return oss.str();
  }

  void publish_raise_command() {
    lx_motor_interfaces::msg::MotorStatus msg;
    msg.mode.assign(64, 0);
    msg.data.assign(64, 0.0);

    const double raw_target = target_positions_.empty() ? 0.0 : target_positions_[0];
    const double limited_target = std::clamp(raw_target, 0.0, 0.2);

    if (std::abs(raw_target - limited_target) > 1e-9) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "raise target %.4f out of range [0.0, 0.2], clamped to %.4f",
                           raw_target, limited_target);
    }

    msg.mode[0] = 1;
    msg.data[0] = limited_target;
    control_pub_->publish(msg);
  }

  void send_control_command() {
    switch (control_target_) {
      case ControlTarget::kRaise:
        publish_raise_command();
        return;
      case ControlTarget::kWaistd:
      case ControlTarget::kRotateHead:
      case ControlTarget::kNodHead:
      case ControlTarget::kUnknown:
      default:
        break;
    }

    if (!initialized_) {
      return;
    }

    lx_motor_interfaces::msg::MotorStatus msg;
    msg.mode.assign(64, 0);
    msg.data.assign(64, 0.0);

    const std::size_t n = std::min<std::size_t>(target_positions_.size(), 64);
    for (std::size_t i = 0; i < n; ++i) {
      const double target = target_positions_[i];
      const double actual = actual_positions_[i];
      const double error_to_target = target - actual;

      if (std::abs(error_to_target) < threshold_) {
        cmd_positions_[i] = target;
        msg.mode[i] = 1;
        msg.data[i] = target;
      } else {
        const double cur = cmd_positions_[i];
        const double diff = target - cur;
        const double remaining_distance = std::abs(error_to_target);

        double adaptive_step = step_rad_;
        if (startup_phase_[i]) {
          const double traveled = std::abs(actual - start_positions_[i]);
          if (traveled < 0.1) {
            const double acceleration_factor = 0.1 + 0.9 * (traveled / 0.1);
            adaptive_step = step_rad_ * acceleration_factor;
          } else {
            startup_phase_[i] = false;
            adaptive_step = std::min(step_rad_, remaining_distance * 0.3);
          }
        } else {
          adaptive_step = std::min(step_rad_, remaining_distance * 0.3);
        }

        const double delta = clamp_step(diff, adaptive_step);
        const double new_cmd = cur + delta;

        cmd_positions_[i] = new_cmd;
        msg.mode[i] = 1;
        msg.data[i] = new_cmd;
      }
    }

    control_pub_->publish(msg);
  }

  void state_callback(const lx_motor_interfaces::msg::MotorStatus::SharedPtr msg) {
    switch (control_target_) {
      case ControlTarget::kRaise:
        return;
      case ControlTarget::kWaistd:
      case ControlTarget::kRotateHead:
      case ControlTarget::kNodHead:
      case ControlTarget::kUnknown:
      default:
        break;
    }

    const std::size_t n = std::min<std::size_t>(target_positions_.size(), 64);

    if (msg->data.size() < n) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "state msg data size=%zu is smaller than target_position size=%zu",
                           msg->data.size(), n);
      return;
    }

    if (!initialized_) {
      for (std::size_t i = 0; i < n; ++i) {
        cmd_positions_[i] = msg->data[i];
        actual_positions_[i] = msg->data[i];
        start_positions_[i] = msg->data[i];
      }
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initialized cmd_positions from current state: %s",
                  vector_to_string(cmd_positions_).c_str());
    }

    for (std::size_t i = 0; i < n; ++i) {
      actual_positions_[i] = msg->data[i];
    }

    for (std::size_t i = 0; i < n; ++i) {
      const double target = target_positions_[i];
      const double current = msg->data[i];
      const double error = target - current;

      RCLCPP_INFO(this->get_logger(), "joint%zu: current=%.4f rad, target=%.4f rad, error=%.4f rad",
                  i, current, target, error);

      if (std::abs(error) < threshold_) {
        if (!reached_flags_[i]) {
          reached_flags_[i] = true;
          RCLCPP_INFO(this->get_logger(), "joint%zu reached target (|error|<%.6f)", i, threshold_);
        }
      } else {
        reached_flags_[i] = false;
      }
    }

    if (!reached_flags_.empty() &&
        std::all_of(reached_flags_.begin(), reached_flags_.begin() + n,
                    [](bool reached) { return reached; })) {
      RCLCPP_INFO(this->get_logger(), "All joints reached target. Exiting.");
      rclcpp::shutdown();
    }
  }

private:
  ControlTarget control_target_{ControlTarget::kWaistd};
  std::string control_target_name_;
  std::string control_topic_;
  std::string state_topic_;
  bool use_step_mode_{true};

  std::vector<double> target_positions_;
  double threshold_{0.01};
  double period_{0.02};
  double step_deg_{0.2};
  double step_rad_{0.0};

  std::vector<bool> reached_flags_;
  std::vector<double> cmd_positions_;
  std::vector<double> actual_positions_;
  std::vector<double> start_positions_;
  std::vector<bool> startup_phase_;

  bool initialized_{false};

  rclcpp::Publisher<lx_motor_interfaces::msg::MotorStatus>::SharedPtr control_pub_;
  rclcpp::Subscription<lx_motor_interfaces::msg::MotorStatus>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiJointController>();
  rclcpp::spin(node);

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
