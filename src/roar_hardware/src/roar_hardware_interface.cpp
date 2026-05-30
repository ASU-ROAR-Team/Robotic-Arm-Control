#include "roar_hardware/roar_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>

// --- GRIPPER TUNING PARAMETERS ---
// Change these to calibrate your servo limits without recompiling the whole workspace
#define GRIPPER_URDF_MIN_M  0.0     // Meters (from URDF)
#define GRIPPER_URDF_MAX_M  0.08    // Meters (from URDF)
#define GRIPPER_SERVO_MIN_DEG 0.0   // Physical Servo Min
#define GRIPPER_SERVO_MAX_DEG 180.0 // Physical Servo Max

namespace roar_hardware
{

hardware_interface::CallbackReturn RoarHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate storage for 8 joints (6 arm joints + 2 gripper joints)
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Set up the internal publisher node
  pub_node_ = std::make_shared<rclcpp::Node>("roar_hardware_interface_node");
  
  // Apply Best Effort QoS profile
  rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();

  // Change topic name to fk_joint_states
  joint_state_pub_ = pub_node_->create_publisher<sensor_msgs::msg::JointState>(
    "fk_joint_states", qos_profile);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoarHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoarHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoarHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RoarHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set initial commands to current states to prevent jerky startups
  for (size_t i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoarHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoarHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // THE MIRROR EFFECT: Because we are running open-loop (no live encoder feedback yet),
  // we must tell MoveIt that the robot successfully reached the commanded positions.
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_states_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoarHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = time;

  const double rad_to_deg = 180.0 / M_PI;

  // 1. Standard Arm Joints (URDF indexes 0, 1, 2, 3)
  double j0_deg = hw_commands_[0] * rad_to_deg;
  double j1_deg = hw_commands_[1] * rad_to_deg;
  double j2_deg = hw_commands_[2] * rad_to_deg;
  double j3_deg = hw_commands_[3] * rad_to_deg;

  // 2. Differential Gear Joints (URDF indexes 4 & 5)
  double theta_4_deg = hw_commands_[4] * rad_to_deg;
  double theta_5_deg = hw_commands_[5] * rad_to_deg;

  // Mechanical reduction = 1
  double diff_motor_1_deg = theta_4_deg + theta_5_deg;
  double diff_motor_2_deg = theta_4_deg - theta_5_deg;

  // 3. Gripper Mapping (MoveIt controls both left [6] and right [7], we just need one)
  double commanded_m = hw_commands_[6]; // Using left_gripper as the reference

  // Clamp the command to URDF limits just in case
  commanded_m = std::max((double)GRIPPER_URDF_MIN_M, std::min(commanded_m, (double)GRIPPER_URDF_MAX_M));

  // Linear interpolation: Meters -> Servo Degrees
  double gripper_servo_deg = GRIPPER_SERVO_MIN_DEG + 
    ((commanded_m - GRIPPER_URDF_MIN_M) / (GRIPPER_URDF_MAX_M - GRIPPER_URDF_MIN_M)) * (GRIPPER_SERVO_MAX_DEG - GRIPPER_SERVO_MIN_DEG);

  // 4. Populate and Publish
  msg.name = {
    "motor_0", "motor_1", "motor_2", "motor_3", 
    "diff_motor_1", "diff_motor_2", "gripper_servo"
  };
  msg.position = {
    j0_deg, j1_deg, j2_deg, j3_deg, 
    diff_motor_1_deg, diff_motor_2_deg, gripper_servo_deg
  };

  joint_state_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

}  // namespace roar_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  roar_hardware::RoarHardwareInterface, hardware_interface::SystemInterface)