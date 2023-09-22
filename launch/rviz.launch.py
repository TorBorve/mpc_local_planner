from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory("mpc_local_planner"), "config", "mpc_viz.rviz")
    print(f'rviz file: {rviz_file}')
    return LaunchDescription([
        launch_ros.actions.Node(
            # namespace= "rviz2", 
            package='rviz2', 
            executable='rviz2', 
            # output='screen', 
            arguments=["-d", rviz_file])])