"""Start the MID-360 driver, spherical preprocessing, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('livox_ros_driver2')
    config_dir = os.path.join(package_share, 'config')

    driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 3},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': os.path.join(config_dir, 'MID360_config.json')},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    preprocessor = Node(
        package='livox_ros_driver2',
        executable='spherical_range_preprocessor.py',
        name='spherical_range_preprocessor',
        output='screen',
        parameters=[
            {'input_topic': '/livox/lidar'},
            {'pooling': 'min'},
            {'max_range': 70.0},
            {'min_range': 0.10},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '--display-config',
            os.path.join(config_dir, 'display_point_cloud_preprocessed_ROS2.rviz'),
        ],
    )

    return LaunchDescription([driver, preprocessor, rviz])
