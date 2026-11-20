"""One-terminal bringup; all motion starts disabled and needs explicit enable."""
import json
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node
from bunker_tag_follower.configuration import validate_follow_configuration


def build_stack(context):
    def value(name):
        from launch.substitutions import LaunchConfiguration
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        raw = value(name).lower()
        if raw not in ('true', 'false'):
            raise RuntimeError(name + ' must be true or false')
        return raw == 'true'

    flags = {name: boolean(name) for name in (
        'hardware', 'centered_forward_mount', 'start_camera', 'start_detector',
        'start_base', 'start_overlay', 'start_follow', 'start_depth_monitor', 'panel')}
    mode = value('mode')
    if mode not in ('handheld', 'mounted') or (flags['hardware'] and mode != 'mounted'):
        raise RuntimeError('Hardware output requires mode:=mounted')
    size = float(value('tag_size'))
    if not 0 < size < 10:
        raise RuntimeError('tag_size must be a positive size in meters')
    config = value('config')
    validate_follow_configuration(config)
    port = value('port_name')
    if flags['start_base']:
        result = subprocess.run(['ip', '-json', '-details', 'link', 'show', port],
                                text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise RuntimeError('CAN interface missing: ' + port + '; connect the adapter first')
        info = json.loads(result.stdout)[0]
        if 'UP' not in info['flags']:
            raise RuntimeError('CAN is down; run: sudo ip link set ' + port + ' up type can bitrate 500000')
        bitrate = info.get('linkinfo', {}).get('info_data', {}).get('bittiming', {}).get('bitrate')
        if bitrate != 500000:
            raise RuntimeError('Expected CAN bitrate 500000; actual: ' + str(bitrate))

    # Avoid duplicating the nodes that users previously launched in separate terminals.
    expected = set()
    for flag, names in {
        'start_camera': {'/camera/camera'}, 'start_detector': {'/apriltag'},
        'start_base': {'/bunker'}, 'start_overlay': {'/apriltag_overlay'},
        'start_follow': {'/tag_follower', '/follow_velocity_guard'},
        'start_depth_monitor': {'/follow_depth_clearance'},
    }.items():
        if flags[flag]:
            expected.update(names)
    if expected:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        probe_context = Context()
        rclpy.init(args=[], context=probe_context)
        probe = rclpy.create_node('follow_bringup_check', context=probe_context)
        probe_executor = SingleThreadedExecutor(context=probe_context)
        probe_executor.add_node(probe)
        try:
            end = time.monotonic()+1.5
            found = set()
            while time.monotonic() < end:
                probe_executor.spin_once(timeout_sec=.05)
                found.update((namespace.rstrip('/')+'/'+name) for name, namespace
                             in probe.get_node_names_and_namespaces())
            duplicates = expected & found
        finally:
            probe_executor.shutdown()
            probe.destroy_node()
            rclpy.shutdown(context=probe_context)
        if duplicates:
            raise RuntimeError('Already running: ' + ', '.join(sorted(duplicates))
                               + '. Stop the old launch terminals or set the corresponding start_*:=false.')

    share = Path(get_package_share_directory('bunker_tag_follower'))
    config = value('config')
    def stop_on_driver_exit(event, launch_context):
        if event.cmd and Path(event.cmd[0]).name in ('astra_camera_node', 'bunker_base_node'):
            return [EmitEvent(event=Shutdown(reason='Camera or base driver exited'))]
        return []

    actions = [RegisterEventHandler(OnProcessExit(on_exit=stop_on_driver_exit))]
    if flags['start_camera']:
        camera = Path(get_package_share_directory('astra_camera')) / 'launch' / 'astra_pro.launch.xml'
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(str(camera))))
    if flags['start_detector']:
        actions.append(Node(
            package='apriltag_ros', executable='apriltag_node', name='apriltag', output='screen',
            parameters=[str(share/'config'/'apriltag_36h11_id0.yaml'), {'size': size}],
            remappings=[('image_rect', '/camera/color/image_raw'),
                        ('camera_info', '/camera/color/camera_info'),
                        ('detections', '/apriltag/detections')],
            on_exit=[EmitEvent(event=Shutdown(reason='AprilTag detector exited'))]))
    if flags['start_base']:
        base = Path(get_package_share_directory('bunker_base')) / 'launch' / 'bunker_base.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base)),
            launch_arguments={'port_name': port, 'is_bunker_mini': value('is_bunker_mini')}.items()))
    if flags['start_overlay']:
        actions.append(Node(package='bunker_tag_follower', executable='tag_overlay',
                            name='apriltag_overlay', output='screen'))
    if flags['start_follow']:
        # Use the same validated follower launch and policy as the standalone entry point.
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch'/'follow.launch.py')),
            launch_arguments={'mode': mode, 'hardware': value('hardware'),
                              'centered_forward_mount': value('centered_forward_mount'),
                              'start_depth_monitor': 'false', 'config': config}.items()))
    if flags['start_depth_monitor']:
        actions.append(Node(package='bunker_tag_follower', executable='depth_clearance',
                            name='follow_depth_clearance', output='screen', parameters=[config],
                            on_exit=[EmitEvent(event=Shutdown(reason='Depth monitor exited'))]))
    if flags['panel']:
        actions.append(Node(package='bunker_tag_follower', executable='control_panel',
                            name='follow_control_panel', output='screen',
                            parameters=[{'hardware': flags['hardware']}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('bunker_tag_follower'))
    defaults = {
        'mode': ('mounted', 'handheld or mounted'),
        'hardware': ('true', 'Enable the hardware output route; motion still starts disabled'),
        'centered_forward_mount': ('true', 'Requires a rigid centered camera facing level forwards'),
        'tag_size': ('0.15', 'Measured black square edge in meters'),
        'port_name': ('can0', 'CAN interface, already configured at 500000'),
        'is_bunker_mini': ('true', 'Use the previously configured Bunker Mini parameter'),
        'start_camera': ('true', 'Start Astra Pro; false to reuse an existing camera'),
        'start_detector': ('true', 'Start AprilTag ID 0 detector'),
        'start_base': ('true', 'Start Bunker base driver'),
        'start_overlay': ('true', 'Start box and axes overlay'),
        'start_follow': ('true', 'Start follower and independent velocity guard'),
        'start_depth_monitor': ('true', 'Start depth proximity and timeout monitoring'),
        'panel': ('true', 'Open the image and enable/stop control window'),
        'config': (str(share/'config'/'follow.yaml'), 'Follower and depth configuration file'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in defaults.items()],
        OpaqueFunction(function=build_stack),
    ])
