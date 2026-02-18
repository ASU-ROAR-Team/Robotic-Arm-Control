#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
import sys, select, termios, tty

class RoarIKJogger(Node):
    def __init__(self):
        super().__init__('roar_ik_jogger')

        # === CONFIGURATION (MATCHING YOUR SRDF) ===
        self.group_name = "arm_controller" 
        self.ee_link = "Link_EE"        # From your SRDF
        self.base_frame = "world"       # From your robot_state_publisher log
        
        # Explicitly set sim time to True to sync with Gazebo
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Connecting to MoveGroup Action Server...")
        self._action_client.wait_for_server()
        
        # Starting point for IK
        self.x, self.y, self.z = 0.2, 0.0, 0.2 
        self.step = 0.02 
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.05, self.keyboard_loop)
        
        self.get_logger().info(f"\nIK JOGGER READY (Using Pilz PTP)"
                               f"\nControls: W/S (X) | A/D (Y) | R/F (Z)"
                               f"\nGroup: {self.group_name} | EE: {self.ee_link}")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_ik_goal(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        
        # MANDATORY FOR PILZ
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = "PTP" 
        
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 1.0
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg() # Critical for sync
        target_pose.pose.position.x = self.x
        target_pose.pose.position.y = self.y
        target_pose.pose.position.z = self.z
        target_pose.pose.orientation.w = 1.0 

        goal_msg.request.goal_constraints.append(self.pose_to_constraints(target_pose))
        
        self.get_logger().info(f"Target: X:{self.x:.2f} Y:{self.y:.2f} Z:{self.z:.2f}")
        self._action_client.send_goal_async(goal_msg)

    def pose_to_constraints(self, pose_stamped):
        c = Constraints()
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = self.ee_link
        pc.constraint_region.primitive_poses.append(pose_stamped.pose)
        from shape_msgs.msg import SolidPrimitive
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.02]))
        pc.weight = 1.0
        c.position_constraints.append(pc)
        return c

    def keyboard_loop(self):
        key = self.get_key()
        if not key: return

        if key == 'w': self.x += self.step
        elif key == 's': self.x -= self.step
        elif key == 'a': self.y += self.step
        elif key == 'd': self.y -= self.step
        elif key == 'r': self.z += self.step
        elif key == 'f': self.z -= self.step
        elif key == '\x03': rclpy.shutdown()
        
        if key in 'wsardf':
            self.send_ik_goal()

def main(args=None):
    rclpy.init(args=args)
    node = RoarIKJogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()