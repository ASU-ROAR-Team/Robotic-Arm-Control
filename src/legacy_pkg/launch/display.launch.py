from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_gui_fk_joint_states = LaunchConfiguration('use_gui_fk_joint_states')

    # 1. Set the correct package and file names
    pkg_path = get_package_share_directory('legacy_pkg')
    urdf_file = os.path.join(pkg_path, 'urdf', 'legacy.urdf')

    # 2. Read the URDF file directly
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # 3. Robot State Publisher (Publishes TF tree)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        condition=IfCondition(use_gui_fk_joint_states)
    )

    robot_state_publisher_node_local = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        condition=UnlessCondition(use_gui_fk_joint_states)
    )

    # 4. Joint State Publisher GUI (Publishes Joint States for the TF tree)
    # CRITICAL FIX: We must pass the robot_description here too!
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        condition=UnlessCondition(use_gui_fk_joint_states)
    )

    fk_bridge = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_path, 'launch', 'fk_joint_state_to_arm_bridge.py'),
            '--publish-joint-states',
        ],
        output='screen',
        condition=IfCondition(use_gui_fk_joint_states),
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui_fk_joint_states',
            default_value='true',
            description='If true, consume GUI joint states from /fk_joint_states'
        ),
        robot_state_publisher_node,
        robot_state_publisher_node_local,
        joint_state_publisher_node,
        fk_bridge,
        rviz_node
    ])