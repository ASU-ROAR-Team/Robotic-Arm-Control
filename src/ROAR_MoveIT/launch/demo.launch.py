import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the Configuration
    moveit_config = MoveItConfigsBuilder("New_URDF", package_name="ROAR_MoveIT").to_moveit_configs()

    # 2. Define the "Brain" (Move Group)
    # This node calculates the path. We force it to use Sim Time.
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    # 3. Define the Visualizer (RViz)
    # This loads your 3D view. We force it to use Sim Time.
    run_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            moveit_config.package_path, "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
    )

    # 4. Return ONLY these two (No Drivers/Spawners!)
    return LaunchDescription([
        run_move_group_node,
        run_rviz_node,
    ])