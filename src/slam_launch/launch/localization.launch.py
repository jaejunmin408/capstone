import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("slam_launch"),   # 내 패키지
            'config',
            'mapper_params_localization.yaml'             # 내가 만든 localization용 yaml
        ),
        description='Full path to the slam_toolbox localization params file'
    )

    map_file = os.path.join( os.path.expanduser('~'), 'maps', 'floor5_test2.yaml' )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )
    
    localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'map_file_name': map_file}
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(lidar_node)
    ld.add_action(localization_node)
    return ld
