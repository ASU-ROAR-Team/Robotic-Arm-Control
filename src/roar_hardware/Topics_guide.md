### Micro-ROS MCU Communication Contract

| MCU Action | Topic Name | Message Type | QoS | Data Structure & Units |
| :--- | :--- | :--- | :--- | :--- |
| **Subscribes (Listens)** | `roar_robot_arm/joint_cmd` | `std_msgs/Float32MultiArray` | Best Effort | 6 Elements: `[j0, j1, j2, j3, diff_m1, diff_m2]` (Degrees) |
| **Subscribes (Listens)** | `roar_robot_ee/joint_cmd` | `std_msgs/Float32` | Best Effort | Single Value: `gripper_servo_deg` (0.0 to 180.0) |
| **Publishes (Sends)** | `roar_robot_arm/joint_feedback` | `std_msgs/Float32MultiArray` | Best Effort | 6 Elements: `[j0, j1, j2, j3, diff_m1, diff_m2]` (Degrees) |
| **Publishes (Sends)** | `roar_robot_ee/joint_feedback` | `std_msgs/Float32` | Best Effort | Single Value: `gripper_servo_deg` (0.0 to 180.0) |