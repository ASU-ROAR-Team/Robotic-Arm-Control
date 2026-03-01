# import the packages 
from launch import LaunchDescription 
from launch_ros.actions import SetParameter 
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess # Changed import
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource 
import os 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description(): 
    use_gui_fk_joint_states = LaunchConfiguration('use_gui_fk_joint_states')
    run_teleop_node = LaunchConfiguration('run_teleop_node')

    gui_fk_arg = DeclareLaunchArgument(
        'use_gui_fk_joint_states',
        default_value='true',
        description='If true, start GUI FK bridge (/fk_joint_states -> arm controller commands)'
    )
    run_teleop_arg = DeclareLaunchArgument(
        'run_teleop_node',
        default_value='true',
        description='If true, run teleop.py in ROS-topic mode for /ik_target_pose and /teleop_command'
    )
    
    # 1. PATH CONFIGURATION
    moveit_pkg_path = get_package_share_directory('legacy_moveit')
    legacy_pkg_path = get_package_share_directory('legacy_pkg')

    # 2. INCLUDE GAZEBO
    lab_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(legacy_pkg_path, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'use_gui_fk_joint_states': use_gui_fk_joint_states}.items()
    ) 
    
    # 3. INCLUDE moveit (without demo stack to avoid duplicate ros2_control/spawners)
    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_path, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    moveit_rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_path, 'launch', 'moveit_rviz.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # 4. STAGGERED STARTUP
    # Give Gazebo + ros2_control enough time to initialize sim clock and controllers,
    # then start MoveIt first and RViz shortly after.
    delay_move_group = TimerAction(
        period=8.0,
        actions=[move_group_node]
    )

    delay_rviz = TimerAction(
        period=11.0,
        actions=[moveit_rviz_node]
    )

    workspace_root = os.path.abspath(os.path.join(moveit_pkg_path, '..', '..', '..', '..'))
    teleop_script = os.path.join(workspace_root, 'src', 'scripts', 'teleop.py')
    teleop_node = ExecuteProcess(
        cmd=['python3', teleop_script],
        output='screen',
        condition=IfCondition(run_teleop_node),
    )
    delay_teleop = TimerAction(
        period=12.0,
        actions=[teleop_node],
    )
    
    return LaunchDescription([ 
        gui_fk_arg,
        run_teleop_arg,
        # Global setting to tell all nodes "Look at the Gazebo Clock, not the Wall Clock"
        SetParameter(name="use_sim_time", value=True), 
        lab_gazebo, 
        delay_move_group,
        delay_rviz,
        delay_teleop,
    ])