#ifndef ROAR_HARDWARE__ROAR_HARDWARE_INTERFACE_HPP_
#define ROAR_HARDWARE__ROAR_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
  // Internal ROS 2 Node for publishing
  std::shared_ptr<rclcpp::Node> pub_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Storage for ros2_control to read/write to
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace roar_hardware

#endif  // ROAR_HARDWARE__ROAR_HARDWARE_INTERFACE_HPP_