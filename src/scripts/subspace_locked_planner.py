#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import threading
import time
import math

# ROS 2 & MoveIt Messages
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# MoveIt IK Service
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

# Optional Collision Guard from package
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

# Minimal joint move threshold in radians (~2.8 degrees)
JOINT_DEADBAND = 0.05

# Minimal Cartesian distance threshold in meters (5 millimeters)
CARTESIAN_POS_TOLERANCE = 0.005  


def apply_quaternion_rotation(q, axis, angle_rad):
    """
    Applies a rotation around a specific axis ('x', 'y', or 'z') to quaternion q.
    """
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
        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)
        
        if HAS_COLLISION_GUARD:
            self.collision_guard = CollisionGuard(self, GROUP_NAME)
        
        self.goal_done = threading.Event()
        self.goal_done.set()

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

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
        """
        Universal Multi-Joint IK Solver: Handles angular distortions caused by 
        manual changes on ANY joint (J0 through J5).
        """
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

        # STEP 1: Direct Attempt (Position + Active Tool Orientation)
        if current_rot is not None:
            pose_stamped.pose.orientation = current_rot
        else:
            pose_stamped.pose.orientation.w = 1.0

        sol = self._call_ik_service(pose_stamped, current_q, timeout_ms=150)
        if sol is not None:
            return sol

        # STEP 2: Exact J0 Radial Yaw Calculation (XY Base Pan Check)
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
                self.get_logger().info(f"Target reached by compensating J0 radial yaw ({math.degrees(delta_j0_yaw):.1f}°)")
                return sol

        # STEP 3: Multi-Axis Rotation Sweep (Handles J1, J2, J3, J4, J5 distortions)
        self.get_logger().warn("Direct & Base-Yaw IK failed. Executing multi-joint orientation search...")
        
        sweep_angles = [-30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -90.0, 90.0, -135.0, 135.0, -180.0, 180.0]
        axes = ['z', 'y', 'x']  # Z = J0/Yaw, Y = J1/J2/J4 Pitch, X = J3/J5 Roll

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
                        self.get_logger().info(f"Found valid IK solution under {axis.upper()}-axis rotation ({deg}°)")
                        return sol

        # STEP 4: Standard Pose Fallbacks (Downward Pitch & Identity)
        self.get_logger().warn("Multi-axis search exhausted. Retrying with canonical orientation fallbacks...")
        fallbacks = [
            # Downward pitch
            (0.0, 0.7071, 0.0, 0.7071),
            # Neutral Identity
            (0.0, 0.0, 0.0, 1.0)
        ]

        for qx, qy, qz, qw in fallbacks:
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            
            sol = self._call_ik_service(pose_stamped, current_q, timeout_ms=150)
            if sol is not None:
                return sol

        return None

    def execute_single_joint_step(self, active_joint_idx, target_angle, frozen_joint_state):
        """
        Executes a 1-DOF move on active_joint_idx via MoveGroup action,
        locking all other 5 joints as static constraints.
        """
        self.goal_done.clear()
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = FRAME_ID
        goal_msg.request.group_name = GROUP_NAME
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 5
        
        constraints = Constraints()
        
        for i, name in enumerate(JOINT_NAMES):
            jc = JointConstraint()
            jc.joint_name = name
            
            if i == active_joint_idx:
                jc.position = float(target_angle)
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
            else:
                jc.position = float(frozen_joint_state[i])
                jc.tolerance_above = 0.005
                jc.tolerance_below = 0.005
                
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
        """Executes sequential 1-DOF joint steps to target XYZ if not already there."""
        current_pos, current_rot = self.get_current_transform()
        if current_pos is None:
            return

        # CARTESIAN DISTANCE CHECK
        distance_to_target = np.sqrt(
            (current_pos.x - target_x)**2 + 
            (current_pos.y - target_y)**2 + 
            (current_pos.z - target_z)**2
        )

        if distance_to_target < CARTESIAN_POS_TOLERANCE:
            print(f"\n>>> ALREADY AT TARGET POSITION (Distance = {distance_to_target*1000:.2f} mm < {CARTESIAN_POS_TOLERANCE*1000:.0f} mm limit). No arm movement needed. <<<\n")
            return

        print(f"\n[Subspace Planner] Moving from ({current_pos.x:.3f}, {current_pos.y:.3f}, {current_pos.z:.3f}) -> Target ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        print(f"Distance to target: {distance_to_target*1000:.1f} mm")
        
        # 1. Compute target IK configuration
        q_target = self.compute_ik_for_target(target_x, target_y, target_z, current_rot)
        if q_target is None:
            print("Target point unreachable or MoveIt IK failed on all orientation fallbacks!")
            return

        # 2. Sequential Execution (Joint 0 -> Joint 5)
        active_moves = 0
        
        for j in range(len(JOINT_NAMES)):
            live_state = self.get_current_joint_array()
            target_angle = q_target[j]
            start_angle = live_state[j]
            delta = abs(target_angle - start_angle)
            
            if delta < JOINT_DEADBAND:
                print(f"Skipping {JOINT_NAMES[j]} (delta = {delta:.3f} rad < deadband threshold)")
                continue
                
            active_moves += 1
            print(f"Step {active_moves}: Moving {JOINT_NAMES[j]} from {start_angle:.3f} to {target_angle:.3f} rad (delta = {delta:.3f} rad)")
            
            self.execute_single_joint_step(j, target_angle, live_state)
            
            self.goal_done.wait()
            self.goal_done.clear()
            time.sleep(0.05)

        print(f">>> SEQUENTIAL EXECUTION COMPLETE ({active_moves} active joint moves executed) <<<\n")


def user_input_thread(node):
    print("-------------------------------------------------")
    print(" Subspace 1-DOF Sequential Planner Active")
    print(" Commands: 'x 0.05'               (Move X relative by +5cm)")
    print("           'z -0.1'               (Move Z relative by -10cm)")
    print("           'abs 0.25 -0.53 0.90'  (Move to Absolute X, Y, Z)")
    print("           'home'                 (Return directly to Home posture)")
    print("           'pos' or 'p'           (Print current X, Y, Z position)")
    print("           'q'                    (Quit)")
    print("-------------------------------------------------")
    
    time.sleep(1)
    
    while rclpy.ok():
        try:
            cmd = input("Enter Command: ").strip().split()
            if not cmd: 
                continue
            
            action = cmd[0].lower()
            if action == 'q':
                print("Quitting...")
                rclpy.shutdown()
                break

            # Print current position helper
            if action in ['pos', 'p']:
                pos, _ = node.get_current_transform()
                if pos:
                    print(f"\nCurrent EE Position ({LINK_NAME} in {FRAME_ID}):")
                    print(f"  X: {pos.x:.4f} m")
                    print(f"  Y: {pos.y:.4f} m")
                    print(f"  Z: {pos.z:.4f} m\n")
                continue

            # Home Posture Command
            if action == 'home':
                home_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                print("\n[Subspace Planner] Executing direct 1-DOF return to Home joint posture...")
                active_moves = 0
                
                for j in range(len(JOINT_NAMES)):
                    live_state = node.get_current_joint_array()
                    target_angle = home_joint_angles[j]
                    start_angle = live_state[j]
                    delta = abs(target_angle - start_angle)
                    
                    if delta < JOINT_DEADBAND:
                        print(f"Skipping {JOINT_NAMES[j]} (delta < deadband)")
                        continue
                        
                    active_moves += 1
                    print(f"Step {active_moves}: Moving {JOINT_NAMES[j]} to {target_angle:.3f} rad...")
                    node.execute_single_joint_step(j, target_angle, live_state)
                    node.goal_done.wait()
                    node.goal_done.clear()
                    time.sleep(0.05)
                    
                print(f">>> HOME POSITION REACHED ({active_moves} joint moves) <<<\n")
                continue

            # Absolute XYZ Target Command
            if action == 'abs':
                if len(cmd) < 4:
                    print("Usage for absolute: abs X Y Z  (e.g., 'abs 0.25 -0.53 0.90')")
                    continue
                tx, ty, tz = float(cmd[1]), float(cmd[2]), float(cmd[3])
                node.send_target_pose(tx, ty, tz)
                continue

            # Relative Delta XYZ Command
            if len(cmd) < 2:
                print("Invalid format. Use: axis amount (e.g., 'x 0.1') or 'abs X Y Z'")
                continue

            val = float(cmd[1])
            pos, _ = node.get_current_transform()
            if pos is None:
                continue

            dx, dy, dz = 0.0, 0.0, 0.0
            if action == 'x': dx = val
            elif action == 'y': dy = val
            elif action == 'z': dz = val
            else:
                print("Unknown axis. Use x, y, z, abs, home, pos, or q.")
                continue

            node.send_target_pose(pos.x + dx, pos.y + dy, pos.z + dz)
            
        except ValueError:
            print("Please enter valid numbers.")
        except Exception as e:
            print(f"Error: {e}")
            break


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