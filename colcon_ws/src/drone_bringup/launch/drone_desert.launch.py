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

    drone_description = get_package_share_directory('drone_description')
    # world_file = os.path.join(drone_description, 'worlds', 'rubicon_world.sdf')
    world_file = os.path.join(drone_description, 'worlds', 'sand.world')
    drone_bringup = get_package_share_directory('drone_bringup')
    mavros_params_file = os.path.join(drone_bringup, 'params', 'sim_mavros_params.yaml')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', '-r', world_file],
            output='screen',
        ),
         Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_params_file],
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=[ '/rgbd_camera/depth_image',
                        '/rgbd_camera/image',
                        ],
            output='screen',
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/world/iris_runway/model/iris_with_depth_camera/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ],
            output='screen'
        ),
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=['/world/challenge/model/iris_with_depth_camera/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        #     ],
        #     output='screen'
        # ),

    ])
