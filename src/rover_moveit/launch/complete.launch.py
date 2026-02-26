# import the packages 
from launch import LaunchDescription 
from launch_ros.actions import SetParameter 
from launch.actions import IncludeLaunchDescription, TimerAction # Changed import
from launch.launch_description_sources import PythonLaunchDescriptionSource 
import os 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description(): 
    
    # 1. PATH CONFIGURATION
    moveit_pkg_path = get_package_share_directory('rover_moveit')
    rover_pkg_path = get_package_share_directory('rover_pkg')

    # 2. INCLUDE GAZEBO
    lab_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(rover_pkg_path, 'launch', 'gazebo.launch.py')
        ])
    ) 
    
    # 3. INCLUDE moveit
    moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_path, 'launch', 'demo.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    ) 
    
    # 4. STABLE DELAY (The Fix)
    # Instead of watching a process, we just wait 5 seconds.
    # This prevents the crash and gives Gazebo time to load.
    delay_moveit = TimerAction(
        period=5.0, 
        actions=[moveit_node]
    )
    
    return LaunchDescription([ 
        # Global setting to tell all nodes "Look at the Gazebo Clock, not the Wall Clock"
        SetParameter(name="use_sim_time", value=True), 
        lab_gazebo, 
        delay_moveit, 
    ])