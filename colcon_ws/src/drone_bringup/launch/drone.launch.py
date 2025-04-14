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

    mavros_params_file = os.path.expanduser('/home/orca4/colcon_ws/src/drone_bringup/params/sim_mavros_params.yaml')
    ardupilot_dir = os.path.expanduser('~/ardupilot')
    arducopter_exe = os.path.join(ardupilot_dir, 'build', 'sitl', 'bin', 'arducopter')
    defaults_file = os.path.join(ardupilot_dir, 'Tools', 'autotest', 'default_params', 'copter.parm')

    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=[
        #         arducopter_exe,
        #         '--model', 'gazebo-iris',      # Specify the drone model
        #         '--defaults', defaults_file,   # The parameter file with default settings
        #         '--home', '33.810313,-118.393867,0.0,0'
        #     ],
        #     output='screen',
        # ),
        ExecuteProcess(
            cmd=[arducopter_exe, '-S', '-w', '-M', 'JSON', '--defaults', defaults_file,
                 '-I0', '--home', '33.810313,-118.39386700000001,0.0,0'],
            output='screen',
        ),
        Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_params_file],
        ),
    ])


# import os
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Paths to your ArduSub defaults file
#     ardupilot_dir = os.path.expanduser('~/ardupilot')
#     ardusub_exe = os.path.join(ardupilot_dir, 'build', 'sitl', 'bin', 'ardusub')
#     ardusub_params_file = os.path.join(ardupilot_dir, 'Tools', 'autotest', 'default_params', 'sub.parm')
#     orca_description_dir = get_package_share_directory('orca_description')

#     world_file = os.path.join(orca_description_dir, 'worlds', 'sand.world')
#     orca_bringup_dir = get_package_share_directory('orca_bringup')
#     mavros_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_mavros_params.yaml')
#     return LaunchDescription([
#         # Launch ArduSub SITL (binary MAVLink)
#         ExecuteProcess(
#             cmd=['gz', 'sim', '-v', '3', '-r', world_file],
#             output='screen',
#             # condition=IfCondition(LaunchConfiguration('gzclient')),
#         ),
#         ExecuteProcess(
#             cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file,
#                  '-I0', '--home', '33.810313,-118.39386700000001,0.0,0'],
#             output='screen',
#         ),
#          Node(
#             package='mavros',
#             executable='mavros_node',
#             output='screen',
#             # mavros_node is actually many nodes, so we can't override the name
#             # name='mavros_node',
#             parameters=[mavros_params_file],
#         ),
#     ])