# Copyright 2023 D-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file demonstrating the rosbag2 Player loaded as a composable node.

Loads the `rosbag2_transport::Player` plugin into a component container and plays back a bag,
configured entirely via ROS parameters - no `ros2 bag play` CLI involved.

Run with:
    # play a bag
    ros2 launch rosbag2_transport composable_player.launch.py bag_uri:=/tmp/bag

    # play only specific topics
    ros2 launch rosbag2_transport composable_player.launch.py \\
        bag_uri:=/tmp/bag topics_to_filter:="['/chatter','/odom']"

Stop with Ctrl+C.
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _to_bool(s):
    return str(s).lower() in ('true', '1', 'yes')


def _build_container(context, *args, **kwargs):
    bag_uri = LaunchConfiguration('bag_uri').perform(context)
    storage_id = LaunchConfiguration('storage_id').perform(context)
    rate = float(LaunchConfiguration('rate').perform(context))
    loop = _to_bool(LaunchConfiguration('loop').perform(context))
    start_paused = _to_bool(LaunchConfiguration('start_paused').perform(context))
    disable_keyboard = _to_bool(LaunchConfiguration('disable_keyboard_controls').perform(context))
    clock_freq = float(LaunchConfiguration('clock_publish_frequency').perform(context))
    topics_raw = LaunchConfiguration('topics_to_filter').perform(context)
    container_exec = LaunchConfiguration('container_executable').perform(context)

    topics_list = yaml.safe_load(topics_raw) if topics_raw else []
    if not isinstance(topics_list, list):
        raise ValueError(
            "topics_to_filter:= must be a YAML list, e.g. topics_to_filter:=\"['/foo','/bar']\". "
            f"Got: {topics_raw!r}")

    parameters = {
        'uri': bag_uri,
        'storage_id': storage_id,
        'rate': rate,
        'loop': loop,
        'start_paused': start_paused,
        'disable_keyboard_controls': disable_keyboard,
        'clock_publish_frequency': clock_freq,
    }
    if topics_list:
        parameters['topics_to_filter'] = topics_list

    return [ComposableNodeContainer(
        package='rclcpp_components',
        executable=container_exec,
        name='rosbag2_player_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='rosbag2_transport',
                plugin='rosbag2_transport::Player',
                name='rosbag2_player',
                parameters=[parameters],
            ),
        ],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_uri', default_value='/tmp/bag',
                              description='Bag directory to play (same as `ros2 bag play` arg).'),
        DeclareLaunchArgument('storage_id', default_value='sqlite3',
                              description='Storage plugin id (sqlite3, mcap, ...).'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Playback rate multiplier.'),
        DeclareLaunchArgument('loop', default_value='false',
                              description='Loop playback.'),
        DeclareLaunchArgument('start_paused', default_value='false',
                              description='Start in paused state.'),
        DeclareLaunchArgument('disable_keyboard_controls', default_value='true',
                              description='Disable keyboard controls (recommended for non-interactive use).'),
        DeclareLaunchArgument('clock_publish_frequency', default_value='0.0',
                              description='Rate in Hz to publish /clock (0 = disabled).'),
        DeclareLaunchArgument('topics_to_filter', default_value='[]',
                              description="YAML list of topics to play, e.g. \"['/foo','/bar']\". "
                                          'Leave as [] to play all.'),
        DeclareLaunchArgument('container_executable', default_value='component_container_mt',
                              description='Component container executable.'),
        OpaqueFunction(function=_build_container),
    ])
