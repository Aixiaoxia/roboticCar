"""Detect AprilTag ID 0 using an already running Astra color camera."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(__file__).resolve().parents[1] / 'config' / 'apriltag_36h11_id0.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'tag_size', description='Measured black outer square edge in meters; excludes white margin'),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('detections_topic', default_value='/apriltag/detections'),
        Node(
            package='apriltag_ros', executable='apriltag_node', name='apriltag',
            output='screen',
            parameters=[str(config), {
                'size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float),
            }],
            remappings=[
                ('image_rect', LaunchConfiguration('image_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
                ('detections', LaunchConfiguration('detections_topic')),
            ],
        ),
    ])
