#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import sys, select, termios, tty

# --- SETTINGS ---
GROUP_NAME = "arm_controller"
BASE_FRAME = "base_link"
TARGET_LINK = "Link_5"  # Moving the wrist center point
STEP_SIZE = 0.05        # 5cm steps
# ----------------

msg = """
---------------------------------------
   ROAR XYZ Teleop (5-DOF Optimized)
---------------------------------------
   w / s : Forward / Backward (+X / -X)
   a / d : Left / Right      (+Y / -Y)
   q / e : Up / Down        (+Z / -Z)

   x     : Exit
---------------------------------------
"""

class XYZMover(Node):
    def __init__(self):
        super().__init__('xyz_mover', 
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info("Connecting to MoveIt Brain...")
        self.action_client.wait_for_server()
        self.get_logger().info("Connected! XYZ mode active.")

    def get_link_pos(self):
        try:
            # Current coordinates of Link_5 relative to base_link
            trans = self.tf_buffer.lookup_transform(BASE_FRAME, TARGET_LINK, rclpy.time.Time())
            return trans.transform.translation
        except:
            return None

    def send_goal(self, target_xyz):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = GROUP_NAME
        # Allow more planning time and attempts for difficult IK queries
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Define Workspace Bounds
        goal_msg.request.workspace_parameters.header.frame_id = BASE_FRAME
        goal_msg.request.workspace_parameters.min_corner.x, goal_msg.request.workspace_parameters.min_corner.y, goal_msg.request.workspace_parameters.min_corner.z = -2.0, -2.0, -2.0
        goal_msg.request.workspace_parameters.max_corner.x, goal_msg.request.workspace_parameters.max_corner.y, goal_msg.request.workspace_parameters.max_corner.z = 2.0, 2.0, 2.0

        # Position Constraint ONLY (The magic fix)
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = TARGET_LINK
        
        vol = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # Use a slightly larger tolerance to help OMPL find reachable states (5cm)
        box.dimensions = [0.05, 0.05, 0.05]
        vol.primitives = [box]
        
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target_xyz.x, target_xyz.y, target_xyz.z
        vol.primitive_poses = [pose.pose]
        
        pc.constraint_region = vol
        pc.weight = 1.0
        c.position_constraints.append(pc)
        
        # WE EXPLICITLY DO NOT ADD ORIENTATION CONSTRAINTS
        
        goal_msg.request.goal_constraints.append(c)
        return self.action_client.send_goal_async(goal_msg)

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = XYZMover()
    print(msg)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = getKey(settings)
            if key == 'x': break
            if key == '': continue

            current_pos = node.get_link_pos()
            if not current_pos: continue

            target = Point()
            target.x, target.y, target.z = current_pos.x, current_pos.y, current_pos.z

            if key == 'w': target.x += STEP_SIZE
            elif key == 's': target.x -= STEP_SIZE
            elif key == 'a': target.y += STEP_SIZE
            elif key == 'd': target.y -= STEP_SIZE
            elif key == 'q': target.z += STEP_SIZE
            elif key == 'e': target.z -= STEP_SIZE
            else: continue

            node.get_logger().info(f"XYZ Request: {target.x:.2f}, {target.y:.2f}, {target.z:.2f}")
            future = node.send_goal(target)
            rclpy.spin_until_future_complete(node, future)

            try:
                goal_handle = future.result()
            except Exception as e:
                node.get_logger().error(f"Failed to send goal: {e}")
                continue

            if not goal_handle.accepted:
                node.get_logger().warn("MoveIt rejected the goal")
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            try:
                result = result_future.result()
                node.get_logger().info(f"Move result: {result}")
            except Exception as e:
                node.get_logger().error(f"Error getting result: {e}")
                continue
            
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()