from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    yaml_file = os.path.join(get_package_share_directory("mpc_local_planner"), "auto_gen.yaml")

    mpc_node = launch_ros.actions.Node(namespace= "mpc_local_planner", 
            package='mpc_local_planner', 
            executable='ff_mpc', 
            output='screen', 
            parameters=[yaml_file])
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('mpc_local_planner'), 'launch', 'rviz.launch.py')
        ])
    )
    return LaunchDescription([
        mpc_node,
        rviz
        ])