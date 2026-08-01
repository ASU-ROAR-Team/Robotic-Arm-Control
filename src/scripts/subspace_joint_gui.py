#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import time
import math
import tkinter as tk
from tkinter import ttk

# ROS 2 & MoveIt Message Interfaces
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState

# === CONFIGURATION MATCHING YOUR WORKING SETUP ===
LINK_NAME = "link_6"
GROUP_NAME = "arm_controller"
FRAME_ID = "world"

JOINT_NAMES = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

# Exact URDF limits (min, max in radians)
JOINT_LIMITS = [
    (-3.1416, 3.1416), # joint_0
    (-2.635,  0.017),  # joint_1
    (-0.017,  3.159),  # joint_2
    (-1.6581, 1.6581), # joint_3
    (-1.6581, 1.6581), # joint_4
    (-1.6581, 1.6581)  # joint_5
]


class MoveGroupSliderNode(Node):
    def __init__(self):
        super().__init__('movegroup_slider_node')
        
        # Action Client matching your working setup
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Joint State Listener
        self.current_joints = {}
        self.joint_state_received = False
        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)
        
        self.execution_lock = threading.Lock()

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos
        if not self.joint_state_received and len(self.current_joints) >= len(JOINT_NAMES):
            self.joint_state_received = True

    def get_current_joint_array(self):
        return [self.current_joints.get(name, 0.0) for name in JOINT_NAMES]

    def execute_single_joint_move(self, active_joint_idx, target_radians):
        """
        Executes a 1-DOF move on active_joint_idx using MoveGroup action,
        locking all other 5 joints to their current positions.
        """
        if not self.execution_lock.acquire(blocking=False):
            self.get_logger().warn("MoveGroup is currently busy executing a trajectory.")
            return False

        try:
            current_state = self.get_current_joint_array()
            
            goal_msg = MoveGroup.Goal()
            goal_msg.request.workspace_parameters.header.frame_id = FRAME_ID
            goal_msg.request.group_name = GROUP_NAME
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.num_planning_attempts = 10
            
            constraints = Constraints()
            
            for i, name in enumerate(JOINT_NAMES):
                jc = JointConstraint()
                jc.joint_name = name
                
                if i == active_joint_idx:
                    # Target angle selected by slider
                    jc.position = float(target_radians)
                    jc.tolerance_above = 0.01
                    jc.tolerance_below = 0.01
                else:
                    # Lock inactive joints to current physical position
                    jc.position = float(current_state[i])
                    jc.tolerance_above = 0.005
                    jc.tolerance_below = 0.005
                    
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
                
            goal_msg.request.goal_constraints.append(constraints)
            
            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("MoveGroup action server ('move_action') not available!")
                self.execution_lock.release()
                return False

            self.get_logger().info(
                f"Sending MoveGroup Goal: {JOINT_NAMES[active_joint_idx]} -> {target_radians:.3f} rad ({math.degrees(target_radians):.1f}°)..."
            )
            
            send_future = self._action_client.send_goal_async(goal_msg)
            
            def goal_response_cb(future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().error("Goal REJECTED by MoveGroup.")
                    self.execution_lock.release()
                    return

                self.get_logger().info("Goal ACCEPTED. Planning & Executing...")
                res_future = goal_handle.get_result_async()
                
                def result_cb(r_future):
                    res = r_future.result().result
                    if res.error_code.val == 1:
                        self.get_logger().info(">>> SUCCESS <<<")
                    else:
                        self.get_logger().error(f">>> FAILED (Error Code: {res.error_code.val}) <<<")
                    self.execution_lock.release()

                res_future.add_done_callback(result_cb)

            send_future.add_done_callback(goal_response_cb)
            return True

        except Exception as e:
            self.get_logger().error(f"Execution Error: {e}")
            if self.execution_lock.locked():
                self.execution_lock.release()
            return False


class SliderMoveGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.root.title("1-DOF Joint Move Panel")
        self.root.geometry("540x460")
        self.root.resizable(False, False)

        style = ttk.Style()
        style.theme_use('clam')

        title_label = ttk.Label(
            root, 
            text="1-DOF Joint Angle Controller", 
            font=("Helvetica", 14, "bold")
        )
        title_label.pack(pady=10)

        self.sliders = []
        self.value_labels = []

        main_frame = ttk.Frame(root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        for i, name in enumerate(JOINT_NAMES):
            frame = ttk.Frame(main_frame)
            frame.pack(fill=tk.X, pady=6)

            # Joint Title
            lbl = ttk.Label(frame, text=f"{name}:", width=10, font=("Helvetica", 10, "bold"))
            lbl.pack(side=tk.LEFT, padx=5)

            min_val, max_val = JOINT_LIMITS[i]
            min_deg, max_deg = math.degrees(min_val), math.degrees(max_val)

            # Angle Selection Slider (Degrees)
            slider = ttk.Scale(
                frame, 
                from_=min_deg, 
                to=max_deg, 
                orient=tk.HORIZONTAL, 
                length=230
            )
            slider.pack(side=tk.LEFT, padx=5)
            self.sliders.append(slider)

            # Angle Readout Label
            val_lbl = ttk.Label(frame, text="0.0° (0.00 rad)", width=18)
            val_lbl.pack(side=tk.LEFT, padx=5)
            self.value_labels.append(val_lbl)

            # "Move" Button per Joint
            btn = ttk.Button(
                frame, 
                text="Move", 
                width=6, 
                command=lambda idx=i: self.on_move_pressed(idx)
            )
            btn.pack(side=tk.LEFT, padx=5)

            # Update text on slider drag
            slider.configure(command=lambda val, idx=i: self.update_label(idx, val))

        # Bottom Frame Controls
        bottom_frame = ttk.Frame(root, padding="10")
        bottom_frame.pack(fill=tk.X)

        sync_btn = ttk.Button(
            bottom_frame, 
            text="Sync Sliders to Robot State", 
            command=self.sync_sliders_to_robot
        )
        sync_btn.pack(side=tk.LEFT, padx=10)

        self.status_lbl = ttk.Label(bottom_frame, text="Waiting for /joint_states...", font=("Helvetica", 9, "italic"))
        self.status_lbl.pack(side=tk.RIGHT, padx=10)

        # Check for joint state initialization
        self.root.after(500, self.initial_sync_check)

    def initial_sync_check(self):
        if self.ros_node.joint_state_received:
            self.sync_sliders_to_robot()
            self.status_lbl.config(text="Ready")
        else:
            self.root.after(500, self.initial_sync_check)

    def update_label(self, idx, val_str):
        deg_val = float(val_str)
        rad_val = math.radians(deg_val)
        self.value_labels[idx].config(text=f"{deg_val:.1f}° ({rad_val:.2f} rad)")

    def sync_sliders_to_robot(self):
        """Sets slider positions to current active robot joint angles."""
        current_rads = self.ros_node.get_current_joint_array()
        for i, rad in enumerate(current_rads):
            deg = math.degrees(rad)
            min_val, max_val = JOINT_LIMITS[i]
            min_deg, max_deg = math.degrees(min_val), math.degrees(max_val)
            
            deg_clamped = max(min_deg, min(max_deg, deg))
            self.sliders[i].set(deg_clamped)
            self.update_label(i, deg_clamped)

    def on_move_pressed(self, joint_idx):
        """Triggers MoveGroup action when the Move button is clicked."""
        deg_val = self.sliders[joint_idx].get()
        target_rad = math.radians(deg_val)
        
        self.status_lbl.config(text=f"Moving {JOINT_NAMES[joint_idx]}...")
        
        # Execute move via ROS node
        success = self.ros_node.execute_single_joint_move(joint_idx, target_rad)
        if not success:
            self.status_lbl.config(text="Node Busy or Error")


def main():
    rclpy.init()
    node = MoveGroupSliderNode()

    # Spin ROS node in background thread so callbacks function
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    # Run Tkinter GUI on main thread
    root = tk.Tk()
    gui = SliderMoveGUI(root, node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()