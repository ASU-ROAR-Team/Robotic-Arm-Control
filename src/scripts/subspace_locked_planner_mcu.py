#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import threading
import time
import math

# Standard ROS 2 Messages
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# MoveIt Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

# Optional Collision Guard
try:
    from collision_guard import CollisionGuard
    HAS_COLLISION_GUARD = True
except ImportError:
    HAS_COLLISION_GUARD = False

# === CONFIGURATION ===
LINK_NAME = "link_6"
GROUP_NAME = "arm_controller"
FRAME_ID = "world"

JOINT_NAMES = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

# Micro-ROS Topics from Contract
ARM_CMD_TOPIC = "roar_robot_arm/joint_cmd"
ARM_FEEDBACK_TOPIC = "roar_robot_arm/joint_feedback"
EE_CMD_TOPIC = "roar_robot_ee/joint_cmd"
EE_FEEDBACK_TOPIC = "roar_robot_ee/joint_feedback"

JOINT_DEADBAND = 0.05  # radians
CARTESIAN_POS_TOLERANCE = 0.005  # meters


def apply_quaternion_rotation(q, axis, angle_rad):
    """Applies a rotation around a specific axis ('x', 'y', or 'z') to quaternion q."""
    half_a = angle_rad / 2.0
    s_a, c_a = math.sin(half_a), math.cos(half_a)
    
    qx_a, qy_a, qz_a, qw_a = 0.0, 0.0, 0.0, c_a
    if axis == 'x': qx_a = s_a
    elif axis == 'y': qy_a = s_a
    elif axis == 'z': qz_a = s_a

    qx_b, qy_b, qz_b, qw_b = q.x, q.y, q.z, q.w

    w = qw_a * qw_b - qx_a * qx_b - qy_a * qy_b - qz_a * qz_b
    x = qw_a * qx_b + qx_a * qw_b + qy_a * qz_b - qz_a * qy_b
    y = qw_a * qy_b - qx_a * qz_b + qy_a * qw_b + qz_a * qx_b
    z = qw_a * qz_b + qx_a * qy_b - qy_a * qx_b + qz_a * qw_b

    return x, y, z, w


class SubspaceSequentialMoveGroup(Node):
    def __init__(self):
        super().__init__('subspace_sequential_movegroup')
        
        # 1. Action Client for MoveGroup
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 2. MoveIt IK Service Client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 3. TF Listener & Joint States
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_joints = {}
        self.mcu_feedback_degrees = [0.0] * 6
        
        # QoS Profile: Best Effort (Matching Micro-ROS Contract)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 4. Publishers to Micro-ROS MCU
        self.arm_cmd_pub = self.create_publisher(Float32MultiArray, ARM_CMD_TOPIC, best_effort_qos)
        self.ee_cmd_pub = self.create_publisher(Float32, EE_CMD_TOPIC, best_effort_qos)
        
        # 5. Subscriptions from Micro-ROS MCU & ROS Joint States
        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)
        self.create_subscription(Float32MultiArray, ARM_FEEDBACK_TOPIC, self._mcu_feedback_cb, best_effort_qos)
        
        if HAS_COLLISION_GUARD:
            self.collision_guard = CollisionGuard(self, GROUP_NAME)
        
        self.goal_done = threading.Event()
        self.goal_done.set()

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def _mcu_feedback_cb(self, msg):
        if len(msg.data) >= 6:
            self.mcu_feedback_degrees = list(msg.data)

    def publish_mcu_arm_command(self, joint_angles_rad):
        """
        Converts joint angles (radians) to MCU contract format (degrees) 
        and publishes to roar_robot_arm/joint_cmd.
        """
        # Convert radians to degrees for j0..j3
        j0_deg = math.degrees(joint_angles_rad[0])
        j1_deg = math.degrees(joint_angles_rad[1])
        j2_deg = math.degrees(joint_angles_rad[2])
        j3_deg = math.degrees(joint_angles_rad[3])
        
        # Kinematic conversion for differential wrist (j4, j5)
        j4_deg = math.degrees(joint_angles_rad[4])
        j5_deg = math.degrees(joint_angles_rad[5])
        
        diff_m1 = j4_deg + j5_deg
        diff_m2 = j4_deg - j5_deg
        
        msg = Float32MultiArray()
        msg.data = [
            float(j0_deg), 
            float(j1_deg), 
            float(j2_deg), 
            float(j3_deg), 
            float(diff_m1), 
            float(diff_m2)
        ]
        self.arm_cmd_pub.publish(msg)
        self.get_logger().info(f"[MCU Out] {ARM_CMD_TOPIC}: {[round(x,2) for x in msg.data]} deg")

    def publish_mcu_gripper_command(self, servo_deg):
        """Publishes gripper servo target (0.0 to 180.0 deg) to roar_robot_ee/joint_cmd."""
        msg = Float32()
        msg.data = float(np.clip(servo_deg, 0.0, 180.0))
        self.ee_cmd_pub.publish(msg)

    def get_current_transform(self):
        """Returns translation and orientation of end-effector in world frame."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                FRAME_ID, 
                LINK_NAME, 
                now, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return trans.transform.translation, trans.transform.rotation
        except Exception as e:
            self.get_logger().error(f"Could not find robot pose via TF: {e}")
            return None, None

    def get_current_joint_array(self):
        """Returns current joint values in URDF joint_0 -> joint_5 order."""
        return [self.current_joints.get(name, 0.0) for name in JOINT_NAMES]

    def _call_ik_service(self, pose_stamped, seed_joint_array, timeout_ms=150):
        """Helper to invoke /compute_ik with custom seed and timeout."""
        request = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = GROUP_NAME
        ik_req.ik_link_name = LINK_NAME
        ik_req.avoid_collisions = True
        ik_req.pose_stamped = pose_stamped
        
        seed_state = RobotState()
        seed_state.joint_state.name = JOINT_NAMES
        seed_state.joint_state.position = [float(v) for v in seed_joint_array]
        ik_req.robot_state = seed_state
        ik_req.timeout.sec = 0
        ik_req.timeout.nanosec = int(timeout_ms * 1e6)
        
        request.ik_request = ik_req
        
        future = self.ik_client.call_async(request)
        while not future.done():
            time.sleep(0.005)
            
        response = future.result()
        if response and response.error_code.val == response.error_code.SUCCESS:
            sol_map = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            return [sol_map[name] for name in JOINT_NAMES]
        return None

    def compute_ik_for_target(self, target_x, target_y, target_z, current_rot):
        """Universal Multi-Joint IK Solver with orientation fallbacks."""
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("MoveIt /compute_ik service unavailable!")
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = FRAME_ID
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(target_x)
        pose_stamped.pose.position.y = float(target_y)
        pose_stamped.pose.position.z = float(target_z)
        
        current_q = self.get_current_joint_array()

        if current_rot is not None:
            pose_stamped.pose.orientation = current_rot
        else:
            pose_stamped.pose.orientation.w = 1.0

        sol = self._call_ik_service(pose_stamped, current_q, timeout_ms=150)
        if sol is not None:
            return sol

        # J0 Radial Yaw Check
        curr_pos, _ = self.get_current_transform()
        if curr_pos is not None and current_rot is not None:
            target_yaw = math.atan2(target_y, target_x)
            current_yaw = math.atan2(curr_pos.y, curr_pos.x)
            delta_j0_yaw = target_yaw - current_yaw
            
            qx, qy, qz, qw = apply_quaternion_rotation(current_rot, 'z', delta_j0_yaw)
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            
            sol = self._call_ik_service(pose_stamped, current_q, timeout_ms=150)
            if sol is not None:
                return sol

        # Multi-Axis Rotation Sweep
        sweep_angles = [-30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -90.0, 90.0]
        axes = ['z', 'y', 'x']

        if current_rot is not None:
            for axis in axes:
                for deg in sweep_angles:
                    rad = math.radians(deg)
                    qx, qy, qz, qw = apply_quaternion_rotation(current_rot, axis, rad)
                    pose_stamped.pose.orientation.x = qx
                    pose_stamped.pose.orientation.y = qy
                    pose_stamped.pose.orientation.z = qz
                    pose_stamped.pose.orientation.w = qw
                    
                    sol = self._call_ik_service(pose_stamped, current_q, timeout_ms=50)
                    if sol is not None:
                        return sol

        return None

    def execute_single_joint_step(self, active_joint_idx, target_angle, frozen_joint_state):
        """Executes 1-DOF step via MoveGroup action and syncs command to MCU."""
        self.goal_done.clear()
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = FRAME_ID
        goal_msg.request.group_name = GROUP_NAME
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 5
        
        constraints = Constraints()
        
        target_state_full = list(frozen_joint_state)
        target_state_full[active_joint_idx] = float(target_angle)
        
        for i, name in enumerate(JOINT_NAMES):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(target_state_full[i])
            jc.tolerance_above = 0.01 if i == active_joint_idx else 0.005
            jc.tolerance_below = 0.01 if i == active_joint_idx else 0.005
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        # Send goal to MoveIt
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Publish corresponding Micro-ROS contract command to MCU
        self.publish_mcu_arm_command(target_state_full)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("1-DOF Step rejected by MoveGroup.")
            self.goal_done.set()
            return
            
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val != 1:
            print(f"1-DOF Step failed with Error Code: {result.error_code.val}")
        self.goal_done.set()

    def send_target_pose(self, target_x, target_y, target_z):
        """Executes sequential 1-DOF joint steps to target XYZ."""
        current_pos, current_rot = self.get_current_transform()
        if current_pos is None:
            return

        distance_to_target = np.sqrt(
            (current_pos.x - target_x)**2 + 
            (current_pos.y - target_y)**2 + 
            (current_pos.z - target_z)**2
        )

        if distance_to_target < CARTESIAN_POS_TOLERANCE:
            print(f"\n>>> ALREADY AT TARGET POSITION ({distance_to_target*1000:.2f} mm). <<<\n")
            return

        q_target = self.compute_ik_for_target(target_x, target_y, target_z, current_rot)
        if q_target is None:
            print("Target point unreachable or MoveIt IK failed!")
            return

        active_moves = 0
        for j in range(len(JOINT_NAMES)):
            live_state = self.get_current_joint_array()
            target_angle = q_target[j]
            start_angle = live_state[j]
            delta = abs(target_angle - start_angle)
            
            if delta < JOINT_DEADBAND:
                continue
                
            active_moves += 1
            print(f"Step {active_moves}: Moving {JOINT_NAMES[j]} to {target_angle:.3f} rad...")
            
            self.execute_single_joint_step(j, target_angle, live_state)
            self.goal_done.wait()
            self.goal_done.clear()
            time.sleep(0.05)

        print(f">>> SEQUENTIAL EXECUTION COMPLETE ({active_moves} moves) <<<\n")


def user_input_thread(node):
    print("-------------------------------------------------")
    print(" Subspace 1-DOF Sequential Planner + Micro-ROS Active")
    print(" Commands: 'abs X Y Z'  (Move arm to absolute position)")
    print("           'grip 90'   (Set gripper servo to 90 degrees)")
    print("           'home'      (Return to Home posture)")
    print("           'pos'       (Print current pose & MCU feedback)")
    print("           'q'         (Quit)")
    print("-------------------------------------------------")
    
    time.sleep(1)
    
    while rclpy.ok():
        try:
            cmd = input("Enter Command: ").strip().split()
            if not cmd: continue
            
            action = cmd[0].lower()
            if action == 'q':
                rclpy.shutdown()
                break

            if action in ['pos', 'p']:
                pos, _ = node.get_current_transform()
                if pos:
                    print(f"\nEE Pose -> X: {pos.x:.4f}m, Y: {pos.y:.4f}m, Z: {pos.z:.4f}m")
                    print(f"MCU Feedback (deg): {[round(v, 2) for v in node.mcu_feedback_degrees]}\n")
                continue

            if action == 'grip':
                deg = float(cmd[1])
                node.publish_mcu_gripper_command(deg)
                print(f"Sent Gripper Cmd: {deg} deg")
                continue

            if action == 'abs':
                tx, ty, tz = float(cmd[1]), float(cmd[2]), float(cmd[3])
                node.send_target_pose(tx, ty, tz)
                continue

        except Exception as e:
            print(f"Error: {e}")


def main():
    rclpy.init()
    node = SubspaceSequentialMoveGroup()
    thread = threading.Thread(target=user_input_thread, args=(node,), daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()