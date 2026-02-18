import os
import tempfile
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. GET DYNAMIC PACKAGE PATHS ---
    pkg_roar_pkg = get_package_share_directory('ROAR_pkg')
    pkg_roar_moveit = get_package_share_directory('ROAR_MoveIT')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- 2. DEFINE PATHS ---
    urdf_file_path = os.path.join(pkg_roar_pkg, 'urdf', 'New_URDF.urdf')
    meshes_path = os.path.join(pkg_roar_pkg, 'meshes')
    controllers_yaml_path = os.path.join(pkg_roar_moveit, 'config', 'ros2_controllers.yaml')

    # --- 3. PROCESS URDF ---
    # We read the URDF and fix the paths
    with open(urdf_file_path, 'r') as file:
        robot_desc_content = file.read()
    
    # Replace package:// with file:// for Gazebo
    robot_desc_content = robot_desc_content.replace('package://ROAR_pkg/meshes', 'file://' + meshes_path)
    robot_desc_content = robot_desc_content.replace('package://ROAR_MoveIT/config/ros2_controllers.yaml', controllers_yaml_path)

    # --- CRITICAL FIX: Write to Temp File ---
    # This prevents the "Syntax error: newline unexpected" crash
    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    tmp_urdf.write(robot_desc_content)
    tmp_urdf.close()
    tmp_urdf_path = tmp_urdf.name

    # --- 4. GAZEBO SIMULATION (FORTRESS MODE) ---
    # We use ros_gz_sim, but with GZ_VERSION=fortress set in terminal, it runs Fortress.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # --- 5. SPAWN THE ROBOT ---
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-name', 'New_ROAR_Arm',
                   '-file', tmp_urdf_path, # Load from file, not string!
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen',
    )

    # --- 6. ROBOT STATE PUBLISHER ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_content, 'use_sim_time': True}]
    )

    # --- 7. BRIDGE (ROS <-> FORTRESS) ---
    # Since we are in Fortress mode, we map to ign.msgs
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ign.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ign.msgs.Model',
        ],
        output='screen'
    )

    # --- 8. CONTROLLERS ---
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller_controller"],
    )

    ee_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["EE_controller_controller"],
    )

    # Delay controllers to make sure robot is spawned first
    delay_controllers = TimerAction(
        period=5.0, 
        actions=[joint_state_broadcaster, arm_controller, ee_controller],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        delay_controllers
    ])