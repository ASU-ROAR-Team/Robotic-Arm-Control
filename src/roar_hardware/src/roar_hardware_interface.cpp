#include "roar_hardware/roar_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>

// --- GRIPPER TUNING PARAMETERS ---
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

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  latest_arm_fb_.resize(6, 0.0);

  pub_node_ = std::make_shared<rclcpp::Node>("roar_hardware_interface_node");
  
  // Best Effort QoS profile for micro-ROS
  rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();

  // Initialize Publishers
  arm_cmd_pub_ = pub_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "roar_robot_arm/joint_cmd", qos_profile);
  ee_cmd_pub_ = pub_node_->create_publisher<std_msgs::msg::Float32>(
    "roar_robot_ee/joint_cmd", qos_profile);

  // Initialize Subscribers
  arm_fb_sub_ = pub_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "roar_robot_arm/joint_feedback", qos_profile,
    std::bind(&RoarHardwareInterface::arm_feedback_cb, this, std::placeholders::_1));
  ee_fb_sub_ = pub_node_->create_subscription<std_msgs::msg::Float32>(
    "roar_robot_ee/joint_feedback", qos_profile,
    std::bind(&RoarHardwareInterface::ee_feedback_cb, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

void RoarHardwareInterface::arm_feedback_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(fb_mutex_);
  if (msg->data.size() >= 6) {
    latest_arm_fb_ = msg->data;
    received_arm_fb_ = true;
  }
}

void RoarHardwareInterface::ee_feedback_cb(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(fb_mutex_);
  latest_ee_fb_ = msg->data;
  received_ee_fb_ = true;
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
  for (size_t i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i];
  }

  // Spin up the background thread to handle incoming micro-ROS feedback
  executor_.add_node(pub_node_);
  executor_thread_ = std::make_unique<std::thread>([this]() {
    executor_.spin();
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoarHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Safely shut down the executor thread
  executor_.cancel();
  if (executor_thread_ && executor_thread_->joinable()) {
    executor_thread_->join();
  }
  executor_.remove_node(pub_node_);
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoarHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<float> arm_fb(6, 0.0);
  float ee_fb = 0.0;
  bool arm_valid, ee_valid;

  // Safely copy the latest data from the subscriber thread
  {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    arm_fb = latest_arm_fb_;
    ee_fb = latest_ee_fb_;
    arm_valid = received_arm_fb_;
    ee_valid = received_ee_fb_;
  }

  const double deg_to_rad = M_PI / 180.0;

  if (arm_valid) {
    // Standard Arm Joints (Degrees -> Radians)
    hw_states_[0] = arm_fb[0] * deg_to_rad;
    hw_states_[1] = arm_fb[1] * deg_to_rad;
    hw_states_[2] = arm_fb[2] * deg_to_rad;
    hw_states_[3] = arm_fb[3] * deg_to_rad;

    // Differential Gear Inverse Kinematics
    double j4_deg = (arm_fb[4] + arm_fb[5]) / 2.0;
    double j5_deg = (arm_fb[4] - arm_fb[5]) / 2.0;
    hw_states_[4] = j4_deg * deg_to_rad;
    hw_states_[5] = j5_deg * deg_to_rad;
  }

  if (ee_valid) {
    // Gripper Mapping (Servo Degrees -> Meters)
    double gripper_m = GRIPPER_URDF_MIN_M + 
      ((ee_fb - GRIPPER_SERVO_MIN_DEG) / (GRIPPER_SERVO_MAX_DEG - GRIPPER_SERVO_MIN_DEG)) * (GRIPPER_URDF_MAX_M - GRIPPER_URDF_MIN_M);

    hw_states_[6] = gripper_m; // left_gripper
    hw_states_[7] = gripper_m; // right_gripper
  } else {
    // Fallback: If no EE feedback yet, keep state mirrored to initial command so MoveIt doesn't jump
    hw_states_[6] = hw_commands_[6];
    hw_states_[7] = hw_commands_[7];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoarHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double rad_to_deg = 180.0 / M_PI;

  // 1. Standard Arm Joints
  float j0_deg = hw_commands_[0] * rad_to_deg;
  float j1_deg = hw_commands_[1] * rad_to_deg;
  float j2_deg = hw_commands_[2] * rad_to_deg;
  float j3_deg = hw_commands_[3] * rad_to_deg;

  // 2. Differential Gear Joints
  double theta_4_deg = hw_commands_[4] * rad_to_deg;
  double theta_5_deg = hw_commands_[5] * rad_to_deg;

  float diff_motor_1_deg = theta_4_deg + theta_5_deg;
  float diff_motor_2_deg = theta_4_deg - theta_5_deg;

  // 3. Populate Arm Command Message
  std_msgs::msg::Float32MultiArray arm_msg;
  arm_msg.data = {j0_deg, j1_deg, j2_deg, j3_deg, diff_motor_1_deg, diff_motor_2_deg};
  arm_cmd_pub_->publish(arm_msg);

  // 4. Gripper Mapping
  double commanded_m = hw_commands_[6];
  commanded_m = std::max((double)GRIPPER_URDF_MIN_M, std::min(commanded_m, (double)GRIPPER_URDF_MAX_M));

  float gripper_servo_deg = GRIPPER_SERVO_MIN_DEG + 
    ((commanded_m - GRIPPER_URDF_MIN_M) / (GRIPPER_URDF_MAX_M - GRIPPER_URDF_MIN_M)) * (GRIPPER_SERVO_MAX_DEG - GRIPPER_SERVO_MIN_DEG);

  // 5. Populate EE Command Message
  std_msgs::msg::Float32 ee_msg;
  ee_msg.data = gripper_servo_deg;
  ee_cmd_pub_->publish(ee_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace roar_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  roar_hardware::RoarHardwareInterface, hardware_interface::SystemInterface)