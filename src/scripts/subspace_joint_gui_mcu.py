#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tkinter as tk
from tkinter import ttk
import numpy as np
import threading
import math

# Standard ROS 2 Messages
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState

# === CONFIGURATION ===
ARM_CMD_TOPIC = "roar_robot_arm/joint_cmd"
ARM_FEEDBACK_TOPIC = "roar_robot_arm/joint_feedback"
EE_CMD_TOPIC = "roar_robot_ee/joint_cmd"
EE_FEEDBACK_TOPIC = "roar_robot_ee/joint_feedback"

JOINT_NAMES = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

# Joint Limits in Degrees (adjust to match your URDF limits if needed)
JOINT_LIMITS_DEG = [
    (-180.0, 180.0), # J0
    (-120.0, 120.0), # J1
    (-150.0, 150.0), # J2
    (-180.0, 180.0), # J3
    (-110.0, 110.0), # J4
    (-180.0, 180.0)  # J5
]


class MicroROSGuiNode(Node):
    def __init__(self):
        super().__init__('subspace_joint_gui_mcu')

        # QoS Profile: Best Effort (Matching Micro-ROS Contract)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Micro-ROS Publishers
        self.arm_cmd_pub = self.create_publisher(Float32MultiArray, ARM_CMD_TOPIC, best_effort_qos)
        self.ee_cmd_pub = self.create_publisher(Float32, EE_CMD_TOPIC, best_effort_qos)

        # Micro-ROS Subscriptions
        self.create_subscription(Float32MultiArray, ARM_FEEDBACK_TOPIC, self._arm_feedback_cb, best_effort_qos)
        self.create_subscription(Float32, EE_FEEDBACK_TOPIC, self._ee_feedback_cb, best_effort_qos)

        # Live feedback states (in degrees)
        self.arm_feedback_deg = [0.0] * 6
        self.ee_feedback_deg = 0.0

    def _arm_feedback_cb(self, msg):
        if len(msg.data) >= 6:
            self.arm_feedback_deg = list(msg.data)

    def _ee_feedback_cb(self, msg):
        self.ee_feedback_deg = float(msg.data)

    def publish_arm_joint_degrees(self, j0_deg, j1_deg, j2_deg, j3_deg, j4_deg, j5_deg):
        """
        Publishes 6-element Float32MultiArray to roar_robot_arm/joint_cmd.
        Calculates differential wrist values: diff_m1 = J4 + J5, diff_m2 = J4 - J5
        """
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

    def publish_gripper_degree(self, servo_deg):
        """Publishes single Float32 to roar_robot_ee/joint_cmd (0.0 to 180.0 deg)."""
        msg = Float32()
        msg.data = float(np.clip(servo_deg, 0.0, 180.0))
        self.ee_cmd_pub.publish(msg)


class ArmControlGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("ROAR Arm & Gripper Micro-ROS GUI")
        self.root.geometry("520x680")

        self.joint_vars = []
        self.joint_labels = []
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')

        main_frame = ttk.Frame(root, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Header
        title = ttk.Label(main_frame, text="Micro-ROS MCU Joint Controller", font=('Helvetica', 14, 'bold'))
        title.pack(pady=5)

        contract_info = ttk.Label(main_frame, text=f"Contract: {ARM_CMD_TOPIC} (Degrees / Best Effort)", font=('Helvetica', 9, 'italic'))
        contract_info.pack(pady=2)

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=10)

        # Arm Joint Sliders (J0 - J5)
        slider_frame = ttk.LabelFrame(main_frame, text=" Arm Joint Controls (Degrees) ", padding="10")
        slider_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        for i in range(6):
            f = ttk.Frame(slider_frame)
            f.pack(fill=tk.X, pady=4)

            min_val, max_val = JOINT_LIMITS_DEG[i]
            lbl_title = ttk.Label(f, text=f"J{i} ({JOINT_NAMES[i]}):", width=18)
            lbl_title.pack(side=tk.LEFT)

            var = tk.DoubleVar(value=0.0)
            self.joint_vars.append(var)

            slider = ttk.Scale(f, from_=min_val, to=max_val, variable=var, orient=tk.HORIZONTAL, command=lambda val, idx=i: self.on_slider_move(idx))
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, px=5)

            val_label = ttk.Label(f, text="0.0°", width=8)
            val_label.pack(side=tk.RIGHT)
            self.joint_labels.append(val_label)

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=10)

        # Gripper Servo Control
        ee_frame = ttk.LabelFrame(main_frame, text=" End-Effector / Gripper (Degrees) ", padding="10")
        ee_frame.pack(fill=tk.X, pady=5)

        ee_inner = ttk.Frame(ee_frame)
        ee_inner.pack(fill=tk.X, pady=4)

        ttk.Label(ee_inner, text="Gripper Servo:", width=18).pack(side=tk.LEFT)
        self.ee_var = tk.DoubleVar(value=0.0)
        ee_slider = ttk.Scale(ee_inner, from_=0.0, to=180.0, variable=self.ee_var, orient=tk.HORIZONTAL, command=self.on_ee_move)
        ee_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, px=5)

        self.ee_label = ttk.Label(ee_inner, text="0.0°", width=8)
        self.ee_label.pack(side=tk.RIGHT)

        # Quick Control Buttons
        btn_frame = ttk.Frame(main_frame, padding="10")
        btn_frame.pack(fill=tk.X, pady=5)

        home_btn = ttk.Button(btn_frame, text="Reset All Joints to 0°", command=self.reset_to_zero)
        home_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, px=5)

        close_grip_btn = ttk.Button(btn_frame, text="Close Gripper (180°)", command=lambda: self.set_gripper(180.0))
        close_grip_btn.pack(side=tk.RIGHT, expand=True, fill=tk.X, px=5)

        # Periodic GUI readout update for feedback
        self.update_feedback_labels()

    def on_slider_move(self, idx):
        val = self.joint_vars[idx].get()
        self.joint_labels[idx].config(text=f"{val:.1f}°")
        self.publish_current_arm_state()

    def on_ee_move(self, val):
        deg = self.ee_var.get()
        self.ee_label.config(text=f"{deg:.1f}°")
        self.node.publish_gripper_degree(deg)

    def set_gripper(self, deg):
        self.ee_var.set(deg)
        self.ee_label.config(text=f"{deg:.1f}°")
        self.node.publish_gripper_degree(deg)

    def publish_current_arm_state(self):
        deg_vals = [v.get() for v in self.joint_vars]
        self.node.publish_arm_joint_degrees(*deg_vals)

    def reset_to_zero(self):
        for i in range(6):
            self.joint_vars[i].set(0.0)
            self.joint_labels[i].config(text="0.0°")
        self.publish_current_arm_state()

    def update_feedback_labels(self):
        """Periodically sync GUI loop."""
        self.root.after(100, self.update_feedback_labels)


def main():
    rclpy.init()
    ros_node = MicroROSGuiNode()

    # Spin ROS node in separate background thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    spin_thread.start()

    # Run Tkinter main loop on main thread
    root = tk.Tk()
    app = ArmControlGUI(root, ros_node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()