# import the packages 
from launch import LaunchDescription 
from launch_ros.actions import SetParameter 
from launch.actions import IncludeLaunchDescription, TimerAction # Changed import
from launch.launch_description_sources import PythonLaunchDescriptionSource 
import os 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description(): 
    
    # 1. PATH CONFIGURATION
    moveit_pkg_path = get_package_share_directory('legacy_moveit')
    legacy_pkg_path = get_package_share_directory('legacy_pkg')

    # 2. INCLUDE GAZEBO
    lab_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(legacy_pkg_path, 'launch', 'gazebo.launch.py')
        ])
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
    
    return LaunchDescription([ 
        # Global setting to tell all nodes "Look at the Gazebo Clock, not the Wall Clock"
        SetParameter(name="use_sim_time", value=True), 
        lab_gazebo, 
        delay_move_group,
        delay_rviz,
    ])