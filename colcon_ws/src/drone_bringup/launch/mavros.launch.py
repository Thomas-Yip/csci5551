import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():

    # mavros_params_file = os.path.expanduser('/home/orca4/colcon_ws/src/drone_bringup/params/sim_mavros_params.yaml')
    drone_bringup = get_package_share_directory('drone_bringup')
    mavros_params_file = os.path.join(drone_bringup, 'params', 'sim_mavros_params.yaml')
    # Paths to your ArduSub defaults file

    return LaunchDescription([
        Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_params_file],
        ),
    ])
