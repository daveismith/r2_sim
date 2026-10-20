import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# The sim with Foxglove as its display: Gazebo runs headless (launch_sim gui:=false) and a
# foxglove_bridge on sim time serves the robot to the Foxglove app.
#
# The bridge defaults to port 8766 because the dev container's `foxglove` service, a
# wall-clock bridge for the live robot, already holds 8765. In Foxglove, connect to
# ws://localhost:8766.
# Regex list for foxglove_bridge's service_whitelist: everything but a real dome device.
NOT_THE_DOME = "['^(?!/r2_dome/).*']"


def generate_launch_description():
    sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('r2_sim'), 'launch', 'launch_sim.launch.py'
                )]), launch_arguments={'gui': 'false',
                                       'world': LaunchConfiguration('world'),
                                       'servo_device': LaunchConfiguration('servo_device'),
                                       'dome_mock_name':
                                           LaunchConfiguration('dome_mock_name')}.items()
    )

    foxglove = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('r2_bringup'), 'launch', 'foxglove.launch.py'
                )]), launch_arguments={'use_sim_time': 'true',
                                       'port': LaunchConfiguration('foxglove_port'),
                                       'name': 'foxglove_bridge_sim',
                                       # a Service Call panel on the sim's bridge must not
                                       # reach a real dome device sharing the graph
                                       'service_whitelist': NOT_THE_DOME}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(get_package_share_directory('r2_sim'), 'worlds', 'obstacles.world'),
            description='World to load'),
        DeclareLaunchArgument(
            'servo_device',
            default_value='sim',
            choices=['sim', 'real'],
            description='See launch_sim.launch.py'),
        DeclareLaunchArgument(
            'dome_mock_name',
            default_value='dome_mock',
            description='See launch_sim.launch.py'),
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8766',
            description='WebSocket port for the Foxglove app'),
        sim,
        foxglove
    ])
