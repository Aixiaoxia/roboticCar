"""Start only following and guarding; existing camera/detector remain independent."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from launch_ros.actions import Node
from bunker_tag_follower.configuration import validate_follow_configuration


def start_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    hardware_text = LaunchConfiguration('hardware').perform(context).lower()
    if mode not in ('handheld', 'mounted') or hardware_text not in ('true', 'false'):
        raise RuntimeError('Use mode:=handheld|mounted and hardware:=true|false')
    hardware = hardware_text == 'true'
    if hardware and mode != 'mounted':
        raise RuntimeError('Handheld mode cannot drive Bunker; use preview until the camera is mounted')
    centered = LaunchConfiguration('centered_forward_mount').perform(context).lower()
    depth = LaunchConfiguration('start_depth_monitor').perform(context).lower()
    if centered not in ('true', 'false') or depth not in ('true', 'false'):
        raise RuntimeError('Boolean arguments must be true or false')
    centered = centered == 'true'
    config = LaunchConfiguration('config').perform(context)
    validate_follow_configuration(config)
    nodes = [
        Node(package='bunker_tag_follower', executable='follower', name='tag_follower',
             output='screen', parameters=[config, {'mode': mode, 'centered_forward_mount': centered,
                                                   'hardware': hardware}],
             on_exit=[EmitEvent(event=Shutdown(reason='Follower exited'))]),
        Node(package='bunker_tag_follower', executable='velocity_guard', name='follow_velocity_guard',
             output='screen', parameters=[config, {'mode': mode, 'hardware': hardware}],
             on_exit=[EmitEvent(event=Shutdown(reason='Velocity guard exited'))]),
    ]
    if depth == 'true':
        nodes.append(Node(package='bunker_tag_follower', executable='depth_clearance',
                          name='follow_depth_clearance', output='screen', parameters=[config]))
    return nodes


def generate_launch_description():
    config = Path(get_package_share_directory('bunker_tag_follower')) / 'config' / 'follow.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='handheld'),
        DeclareLaunchArgument('hardware', default_value='false'),
        DeclareLaunchArgument('centered_forward_mount', default_value='false'),
        DeclareLaunchArgument('start_depth_monitor', default_value='false'),
        DeclareLaunchArgument('config', default_value=str(config)),
        OpaqueFunction(function=start_nodes),
    ])
