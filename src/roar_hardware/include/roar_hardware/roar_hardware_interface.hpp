#ifndef ROAR_HARDWARE__ROAR_HARDWARE_INTERFACE_HPP_
#define ROAR_HARDWARE__ROAR_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// New Message Types
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

namespace roar_hardware
{
class RoarHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Internal ROS 2 Node and Executor
  std::shared_ptr<rclcpp::Node> pub_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<std::thread> executor_thread_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ee_cmd_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_fb_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ee_fb_sub_;

  // Callbacks
  void arm_feedback_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void ee_feedback_cb(const std_msgs::msg::Float32::SharedPtr msg);

  // Storage for ros2_control
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // Thread-safe feedback storage
  std::mutex fb_mutex_;
  std::vector<float> latest_arm_fb_;
  float latest_ee_fb_ = 0.0;
  bool received_arm_fb_ = false;
  bool received_ee_fb_ = false;
};

}  // namespace roar_hardware

#endif  // ROAR_HARDWARE__ROAR_HARDWARE_INTERFACE_HPP_