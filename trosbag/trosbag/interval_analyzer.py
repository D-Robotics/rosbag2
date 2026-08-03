# Copyright 2026 D-Robotics
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

"""Analyze header.stamp intervals of ROS2 bag messages (TROS version)."""

import os
import struct
import sys
from collections import defaultdict

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print('ERROR: rosbag2_py, rclpy, and rosidl_runtime_py are required.')
    print("Source your ROS2 workspace first: source install/setup.bash")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    print('ERROR: matplotlib and numpy are required.')
    print('Install with: pip3 install matplotlib numpy')
    sys.exit(1)


def parse_header_stamp_from_cdr(cdr_bytes):
    """
    Parse header.stamp from CDR serialized bytes.

    CDR encapsulation header: 4 bytes (00 01 00 00 for CDR_BE, 00 00 00 00 for CDR_LE)
    Then the message fields follow in order.

    For a ROS2 message with a Header as the first field:
      - Header: stamp (sec: int32, nanosec: uint32) + frame_id (string)

    Returns timestamp in nanoseconds, or None if parsing fails.
    """
    if len(cdr_bytes) < 12:
        return None

    endian_flag = cdr_bytes[1]

    if endian_flag == 0x00:
        fmt_sec = '>i'
        fmt_nsec = '>I'
    elif endian_flag == 0x01:
        fmt_sec = '<i'
        fmt_nsec = '<I'
    else:
        return None

    try:
        offset = 4  # after CDR header
        sec = struct.unpack_from(fmt_sec, cdr_bytes, offset)[0]
        nanosec = struct.unpack_from(fmt_nsec, cdr_bytes, offset + 4)[0]
        return sec * 1_000_000_000 + nanosec
    except (struct.error, ValueError):
        return None


def get_header_stamp(topic_type, cdr_bytes):
    """
    Get header.stamp from a ROS2 message.

    Returns timestamp in nanoseconds, or None.
    """
    try:
        msg_cls = get_message(topic_type)
        if msg_cls is None:
            return parse_header_stamp_from_cdr(cdr_bytes)

        msg = deserialize_message(cdr_bytes, msg_cls)

        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            stamp = msg.header.stamp
            return stamp.sec * 1_000_000_000 + stamp.nanosec

        return None
    except Exception:
        return parse_header_stamp_from_cdr(cdr_bytes)


def open_bag(bag_path, storage_id='mcap'):
    """Open a ROS2 bag file for reading."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def analyze_bag(bag_path, topic_filter=None, storage_id='mcap'):
    """Read all messages and compute per-topic header timestamp intervals."""
    reader = open_bag(bag_path, storage_id=storage_id)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic_filter:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topic_filter)))

    topic_stamps = defaultdict(list)
    total_msgs = 0
    no_header_topics = set()

    print(f'Reading bag: {bag_path}')
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        total_msgs += 1

        header_stamp = get_header_stamp(type_map.get(topic, ''), data)
        if header_stamp is not None:
            topic_stamps[topic].append(header_stamp)
        else:
            if topic not in no_header_topics:
                no_header_topics.add(topic)
                print(f"  [WARN] Topic '{topic}' ({type_map.get(topic, '?')}) has no header, "
                      f'using bag timestamp instead')
            topic_stamps[topic].append(bag_stamp)

        if total_msgs % 10000 == 0:
            print(f'  Read {total_msgs} messages...', end='\r')

    print(f'  Read {total_msgs} messages total.          ')

    topic_intervals = {}
    for topic, stamps in topic_stamps.items():
        if len(stamps) < 2:
            continue
        stamps_sorted = sorted(stamps)
        intervals = []
        for i in range(1, len(stamps_sorted)):
            dt = stamps_sorted[i] - stamps_sorted[i-1]
            if dt >= 0:
                intervals.append(dt)
        topic_intervals[topic] = intervals

    return topic_intervals, type_map


def print_summary(topic_intervals, type_map):
    """Print a text summary of all topics."""
    print('\n' + '=' * 70)
    print(f"{'Topic':<45} {'Msgs':>6} {'Min(ms)':>10} {'Mean(ms)':>10} {'Max(ms)':>10}")
    print('-' * 70)

    sorted_topics = sorted(topic_intervals.items(),
                           key=lambda x: np.mean(x[1]) if x[1] else 0)

    for topic, intervals in sorted_topics:
        if not intervals:
            continue
        arr = np.array(intervals) / 1e6
        print(f"{topic:<45} {len(intervals)+1:>6} {arr.min():>10.3f} {arr.mean():>10.3f} "
              f"{arr.max():>10.3f}")

    print('=' * 70)


def plot_all_topics(topic_intervals, type_map, output_path):
    """Plot all topic interval scatters in one figure (one subplot per topic)."""
    topics = sorted(topic_intervals.keys())
    n = len(topics)
    if n == 0:
        return

    fig, axes = plt.subplots(n, 1, figsize=(14, 3 * n), sharex=False)
    if n == 1:
        axes = [axes]

    for ax, topic in zip(axes, topics):
        intervals = topic_intervals[topic]
        intervals_ms = np.array(intervals) / 1e6
        x = np.arange(len(intervals_ms))

        ax.scatter(x, intervals_ms, s=0.5, alpha=0.6, color='steelblue')
        ax.set_ylabel('Interval (ms)')
        type_name = type_map.get(topic, '?')
        ax.set_title(f'{topic}  ({type_name}, {len(intervals)+1} msgs)', fontsize=9)

        stats_text = (
            f'Min: {intervals_ms.min():.3f} ms\n'
            f'Mean: {intervals_ms.mean():.3f} ms\n'
            f'Std: {intervals_ms.std():.3f} ms\n'
            f'Max: {intervals_ms.max():.3f} ms'
        )
        ax.text(0.97, 0.97, stats_text, transform=ax.transAxes, fontsize=7,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    axes[-1].set_xlabel('Message index')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)


def run(args):
    """Entry point used by the `interval` verb."""
    bag_path = os.path.abspath(args.bag_path)
    if not os.path.exists(bag_path):
        print(f'ERROR: Path does not exist: {bag_path}')
        return 1

    topic_intervals, type_map = analyze_bag(
        bag_path, topic_filter=args.topics, storage_id=args.storage)

    if not topic_intervals:
        print('No messages with intervals found.')
        return 0

    print_summary(topic_intervals, type_map)

    if args.no_plot:
        return 0

    output_dir = args.output_dir or os.path.join(
        os.path.dirname(bag_path), 'bag_interval_plots')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'interval_scatter.png')

    print(f'\nGenerating plot: {output_path}')
    plot_all_topics(topic_intervals, type_map, output_path)
    print('Done!')
    return 0
