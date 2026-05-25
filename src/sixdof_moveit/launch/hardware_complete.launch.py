# import the packages
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def software_gl_actions():
    if os.environ.get('SIXDOF_FORCE_SOFTWARE_GL', '1').lower() in {'0', 'false', 'no', 'off'}:
        return []
    return [
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        SetEnvironmentVariable(name='QT_OPENGL', value='software'),
        SetEnvironmentVariable(name='QT_X11_NO_MITSHM', value='1'),
    ]


def generate_launch_description():
    moveit_pkg_path = get_package_share_directory('sixdof_moveit')
    sixdof_pkg_path = get_package_share_directory('sixdof_pkg')

    # 1. Parse the Xacro file with hardware arguments enabled
    xacro_file = os.path.join(sixdof_pkg_path, 'urdf', 'roar.xacro')
    robot_description_content = Command(
        ['xacro ', xacro_file, ' use_gazebo:=false use_hardware:=true']
    )
    robot_description = {'robot_description': robot_description_content}

    # 2. Controller Manager Configuration
    ros2_controllers_path = os.path.join(moveit_pkg_path, 'config', 'ros2_controllers.yaml')
    
    # 3. Core ROS 2 Nodes (Replacing Gazebo's built-in nodes)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output='both',
    )

    # 4. Controller Spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller_controller', '--controller-manager', '/controller_manager'],
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller_controller', '--controller-manager', '/controller_manager'],
    )

    # 5. MoveIt and RViz 
    # NOTE: use_sim_time is explicitly set to 'false' for physical hardware
    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_path, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    moveit_rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_path, 'launch', 'moveit_rviz.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 6. Timers (Adjusted for hardware boot sequence)
    # Give the controller manager a moment to load before spawning controllers
    delay_spawners = TimerAction(
        period=2.0,
        actions=[jsb_spawner, arm_controller_spawner, hand_controller_spawner]
    )

    delay_move_group = TimerAction(
        period=6.0,
        actions=[move_group_node]
    )

    # I kept the delayed RViz here, but remember you can remove this entirely 
    # if you use your start_complete_stack.py orchestration script to launch it!
    delay_rviz = TimerAction(
        period=15.0,
        actions=[moveit_rviz_node]
    )

    return LaunchDescription([
        # Force software GL if needed
        # *software_gl_actions(),
        
        # CRITICAL: Hardware must use the system wall-clock, not a simulation clock
        SetParameter(name='use_sim_time', value=False),

        # Launch hardware stack
        robot_state_publisher_node,
        controller_manager_node,
        
        # Delayed actions
        delay_spawners,
        delay_move_group,
        delay_rviz,
    ])