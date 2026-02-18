import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. GET DYNAMIC PACKAGE PATHS ---
    pkg_roar_pkg = get_package_share_directory('ROAR_pkg')
    pkg_roar_moveit = get_package_share_directory('ROAR_MoveIT')
    pkg_ros_ign = get_package_share_directory('ros_ign_gazebo')

    # --- 2. DEFINE PATHS ---
    # The raw URDF file
    urdf_file_path = os.path.join(pkg_roar_pkg, 'urdf', 'New_URDF.urdf')
    
    # The real path to the meshes (for replacement)
    meshes_path = os.path.join(pkg_roar_pkg, 'meshes')
    
    # The real path to the controllers (for replacement)
    controllers_yaml_path = os.path.join(pkg_roar_moveit, 'config', 'ros2_controllers.yaml')

    # --- 3. PROCESS URDF (THE "FIND AND REPLACE" TRICK) ---
    # We read the URDF and replace the "package://" placeholders with real paths
    with open(urdf_file_path, 'r') as file:
        robot_desc_content = file.read()
    
    # Replace mesh paths
    robot_desc_content = robot_desc_content.replace(
        'package://ROAR_pkg/meshes', 
        'file://' + meshes_path
    )
    
    # Replace controller config path
    robot_desc_content = robot_desc_content.replace(
        'package://ROAR_MoveIT/config/ros2_controllers.yaml', 
        controllers_yaml_path
    )

    # --- 4. GAZEBO SIMULATION ---
    # Launch Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': '-r empty.sdf'}.items(),
    )

    # --- 5. SPAWN THE ROBOT ---
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-name', 'New_ROAR_Arm',
                   '-string', robot_desc_content, # Pass the processed string directly!
                   '-x', '0', '-y', '0', '-z', '0.1'], # Lift it slightly so it doesn't clip floor
        output='screen',
    )

    # --- 6. ROBOT STATE PUBLISHER ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_content, 'use_sim_time': True}]
    )

    # --- 7. BRIDGE (ROS <-> GAZEBO) ---
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ign.msgs.Clock]',
            '/joint_states@sensor_msgs/msg/JointState[ign.msgs.Model]',
        ],
        output='screen'
    )

    # --- 8. CONTROLLERS (WITH DELAY!) ---
    # We must wait for the robot to spawn before loading controllers, or they will crash.
    
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

    # Delay controllers slightly to ensure the robot is spawned in Gazebo
    delay_controllers = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster, arm_controller, ee_controller],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        delay_controllers
    ])