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

"""Launch file demonstrating the rosbag2 Recorder loaded as a composable node.

Loads the `rosbag2_transport::Recorder` plugin into a component container and configures it
entirely via ROS parameters - no `ros2 bag record` CLI involved.

`bag_uri` is the parent folder where the bag is created. When `bag_name` is not given, a name
is auto-generated using the same format as `ros2 bag record`'s default
(`rosbag2_%Y_%m_%d-%H_%M_%S`), so the final uri passed to the Recorder is
`<bag_uri>/<bag_name>`.

Run with:
    # record all topics (default) into <bag_uri>/rosbag2_<timestamp>
    ros2 launch rosbag2_transport composable_recorder.launch.py bag_uri:=/tmp/bags

    # record a specific topic list (YAML-quoted list string)
    ros2 launch rosbag2_transport composable_recorder.launch.py \\
        bag_uri:=/tmp/bags all:=false topics:="['/chatter','/odom']"

    # use an explicit bag name instead of the auto-generated timestamp
    ros2 launch rosbag2_transport composable_recorder.launch.py \\
        bag_uri:=/tmp/bags bag_name:=test_run

Stop with Ctrl+C (the container tears down the recorder, flushing and closing the bag).
"""

import datetime
import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def _to_bool(s):
    return str(s).lower() in ('true', '1', 'yes')


def _build_container(context, *args, **kwargs):
    """Build either a ComposableNodeContainer or a LoadComposableNodes action.

    When `container_name` is set (non-empty), the Recorder is loaded into an
    already-running container with that name via LoadComposableNodes. When
    empty, a fresh container is spawned with ComposableNodeContainer.

    The Recorder declares `topics` as a string-array with an empty default. launch_ros rejects
    an empty sequence passed as a parameter, so we omit the key entirely when the user did not
    override it and let the Recorder use its own default.

    Note: LaunchConfiguration.perform() returns a string, so bool/int params must be coerced
    back to their real types before handing them to ComposableNode - otherwise rclcpp rejects
    them with "Wrong parameter type".
    """
    bag_uri_parent = LaunchConfiguration('bag_uri').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context).strip()
    storage_id = LaunchConfiguration('storage_id').perform(context)
    all_topics = _to_bool(LaunchConfiguration('all').perform(context))
    topics_raw = LaunchConfiguration('topics').perform(context)
    regex = LaunchConfiguration('regex').perform(context)
    start_paused = _to_bool(LaunchConfiguration('start_paused').perform(context))
    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    no_discovery = _to_bool(LaunchConfiguration('no_discovery').perform(context))
    delay = int(_to_bool(LaunchConfiguration('delay').perform(context)))
    container_exec = LaunchConfiguration('container_executable').perform(context)
    container_name = LaunchConfiguration('container_name').perform(context).strip()

    topics_list = yaml.safe_load(topics_raw) if topics_raw else []
    if not isinstance(topics_list, list):
        raise ValueError(
            "topics:= must be a YAML list, e.g. topics:=\"['/foo','/bar']\". "
            f"Got: {topics_raw!r}")

    # Coerce numeric args with a clear error on bad input instead of a raw Python traceback.
    try:
        max_cache_size = int(LaunchConfiguration('max_cache_size').perform(context))
    except ValueError as e:
        raise ValueError(
            f"max_cache_size:= must be an integer (bytes). Got: "
            f"{LaunchConfiguration('max_cache_size').perform(context)!r}") from e

    # bag_uri is the parent folder; the bag itself goes into a subfolder named
    # bag_name (if given) or the same `rosbag2_%Y_%m_%d-%H_%M_%S` default that
    # `ros2 bag record` uses when no -o is supplied.
    if not bag_name:
        bag_name = datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
    bag_uri = os.path.join(bag_uri_parent, bag_name)
    if os.path.exists(bag_uri):
        raise ValueError(
            f"Output path already exists: {bag_uri!r}. "
            "Pick a different bag_name or remove the existing directory.")

    parameters = {
        'uri': bag_uri,
        'storage_id': storage_id,
        'all': all_topics,
        'regex': regex,
        'start_paused': start_paused,
        'use_sim_time': use_sim_time,
        'is_discovery_disabled': no_discovery,
        'max_cache_size': max_cache_size,
        'delay': delay,
    }
    if topics_list:  # only inject when non-empty; launch_ros rejects empty sequences
        parameters['topics'] = topics_list

    recorder_node = ComposableNode(
        package='rosbag2_transport',
        plugin='rosbag2_transport::Recorder',
        name='rosbag2_recorder',
        parameters=[parameters],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    if container_name:
        # Load into an existing container (e.g. tros_container from bringup)
        return [LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[recorder_node],
        )]

    return [ComposableNodeContainer(
        package='rclcpp_components',
        executable=container_exec,
        name='rosbag2_recorder_container',
        namespace='',
        composable_node_descriptions=[recorder_node],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_uri', default_value='/tmp',
                              description='Parent directory where the bag is created. '
                                          'The bag is written to <bag_uri>/<bag_name>, with '
                                          'bag_name defaulting to the same '
                                          '`rosbag2_%Y_%m_%d-%H_%M_%S` timestamp format that '
                                          '`ros2 bag record` uses when no -o is supplied.'),
        DeclareLaunchArgument('bag_name', default_value='',
                              description='Bag directory name placed under bag_uri. When empty '
                                          '(default), a `rosbag2_%Y_%m_%d-%H_%M_%S` name is '
                                          'auto-generated.'),
        DeclareLaunchArgument('storage_id', default_value='sqlite3',
                              description='Storage plugin id (sqlite3, mcap, ...).'),
        DeclareLaunchArgument('all', default_value='true',
                              description='Record all topics (same as `ros2 bag record -a`). '
                                          'Set all:=false when using topics:= or regex:=.'),
        DeclareLaunchArgument('topics', default_value='[]',
                              description="YAML list of topics, e.g. \"['/foo', '/bar']\". "
                                          'Mutually exclusive with all:=true. Leave as [] to '
                                          'record all (with all:=true) or use regex.'),
        DeclareLaunchArgument('regex', default_value='',
                              description='Record topics matching this regex.'),
        DeclareLaunchArgument('start_paused', default_value='false',
                              description='Start in paused state.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock for message timestamps.'),
        DeclareLaunchArgument('no_discovery', default_value='false',
                              description='Disable topic discovery (only record topics present '
                                          'at startup).'),
        DeclareLaunchArgument('max_cache_size', default_value='104857600',
                              description='Per-buffer cache size in bytes (default 100 MiB).'),
        DeclareLaunchArgument('delay', default_value='true',
                              description='Enable delayed cache consumption (d-robotics perf '
                                          'optimization). Set to false to flush on every message.'),
        DeclareLaunchArgument('container_executable', default_value='component_container_mt',
                              description='Component container executable. Used only when '
                                          'container_name is empty. Use component_container_mt '
                                          '(default) or component_container_isolated - the single-threaded '
                                          'component_container has a known unload race that can crash on '
                                          'ros2 component unload.'),
        DeclareLaunchArgument('container_name', default_value='',
                              description='Existing container name to load the Recorder into. '
                                          'When set (non-empty), uses LoadComposableNodes against that '
                                          'container instead of spawning a new ComposableNodeContainer. '
                                          'Empty (default) spawns a dedicated rosbag2_recorder_container.'),
        OpaqueFunction(function=_build_container),
    ])
