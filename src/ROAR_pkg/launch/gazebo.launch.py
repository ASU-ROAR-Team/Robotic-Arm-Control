from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package Directories
    pkg_desc = get_package_share_directory('ROAR_pkg')
    pkg_moveit = get_package_share_directory('ROAR_MoveIT')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # 1. Setup Paths
    urdf_path = os.path.join(pkg_desc, 'urdf', 'New_URDF.urdf') 
    
    # Absolute path to the YAML file in the MoveIT package
    controller_yaml_absolute = os.path.join(pkg_moveit, "config", "ros2_controllers.yaml")
    
    # Absolute path to the meshes
    mesh_absolute_path = os.path.join(pkg_desc, 'meshes')

    # 2. Process URDF
    # We read the file and replace the 'package://' paths with absolute paths
    # so Gazebo/ros2_control can find them without error.
    with open(urdf_path, 'r') as urdf_file:
        robot_desc_raw = urdf_file.read()

    # The strings below MUST match exactly what is written in your New_URDF.urdf
    robot_description = robot_desc_raw.replace(
        "package://ROAR_pkg/meshes",
        mesh_absolute_path
    ).replace(
        "package://ROAR_MoveIT/config/ros2_controllers.yaml",
        controller_yaml_absolute
    )

    # 3. Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )
    
    # 5. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot_New_URDF', 
                   '-topic', '/robot_description'],
        output='screen',
    )

    # 6. Bridge
    # Note: Using 'robot_New_URDF' to match the spawn name
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/robot_New_URDF/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/robot_New_URDF/joint_state', 'joint_states'),
        ],
        output='screen'
    )

    # 7. Spawners
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # Corrected name to match your YAML
    load_arm_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller_controller"],
        output="screen"
    )

    # Corrected name to match your YAML
    load_hand_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["EE_controller_controller"],
        output="screen"
    )
    

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        bridge,
        load_joint_state_broadcaster,
        load_arm_group_controller,
        load_hand_group_controller,
    ])