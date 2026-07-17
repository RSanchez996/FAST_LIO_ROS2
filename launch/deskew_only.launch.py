import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('fast_lio')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    output_topic = LaunchConfiguration('output_topic')
    lidar_frame = LaunchConfiguration('lidar_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(package_share, 'config'),
            description='Directory containing the FAST-LIO parameter file',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='mid360_deskew_only.yaml',
            description='FAST-LIO deskew-only parameter file',
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/livox/lidar',
            description='Livox CustomMsg input topic',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/livox/imu',
            description='MID-360 IMU input topic',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/cloud_deskewed',
            description='Deskewed sensor_msgs/PointCloud2 output topic',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='livox_frame',
            description='Physical frame of the deskewed point coordinates',
        ),
        LogInfo(
            msg=[
                '[fast_lio] Deskew only: /livox/lidar must be ',
                'livox_ros_driver2/msg/CustomMsg; output=',
                output_topic,
                ', frame=',
                lidar_frame,
            ],
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_deskew',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_path, config_file]),
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time,
                        value_type=bool,
                    ),
                    # Force the safe interface contract even if another
                    # compatible MID-360 config file is selected.
                    'processing.deskew_only': True,
                    'preprocess.lidar_type': 1,
                    'common.lid_topic': lidar_topic,
                    'common.imu_topic': imu_topic,
                    'publish.deskewed_topic': output_topic,
                    'frames.lidar_frame': lidar_frame,
                },
            ],
        ),
    ])
