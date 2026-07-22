# rosbag2
![License](https://img.shields.io/github/license/ros2/rosbag2)
[![GitHub Action Status](https://github.com/ros2/rosbag2/workflows/Test%20rosbag2/badge.svg)](https://github.com/ros2/rosbag2/actions)

Repository for implementing rosbag2 as described in its corresponding [design article](https://github.com/ros2/design/blob/ros2bags/articles/rosbags.md).

## D-Robotics 增强特性

本分支（`humble-d-robotics`）基于官方 rosbag2 humble，增加了以下性能与功能增强。所有增强都向后兼容，原有 `ros2 bag record` / `ros2 bag play` 用法完全不受影响。

### 1. 多线程 executor 录制（默认开启）

录制时使用 `MultiThreadedExecutor`，并为每个 topic 创建独立的 `MutuallyExclusive` callback group，不同 topic 的订阅回调可并发执行，高负载下不再互相阻塞。

- **无需任何 flag**，`ros2 bag record` 默认即走多线程路径。
- `Writer::write` 内部加锁，保证多线程并发落盘安全。

### 2. 消息缓存延迟写（msg cache delay write）

通过攒批写盘减少 IO 次数：消息先进缓存，等缓存半满（或超时）才通知消费者线程刷盘，而非每条消息都刷。

| CLI flag | 默认 | 说明 |
|---|---|---|
| `--no-delay` | 关（即默认开 delay） | 关闭延迟写，每条消息立即刷盘（低延迟场景） |
| `--delay-timeout-ms <int>` | `200` | delay 模式下消费者超时刷盘毫秒数 |

```bash
# 默认开 delay（半满刷盘 + 200ms 超时）
ros2 bag record -a -o /tmp/bag

# 自定义超时 100ms
ros2 bag record -a -o /tmp/bag --delay-timeout-ms 100

# 关闭 delay，每条都刷
ros2 bag record -o /tmp/bag --no-delay /chatter
```

> 仅在非 snapshot 模式且 `max_cache_size > 0` 时有意义；cache 关闭时直接写盘，delay 无效。

### 3. 回放路径减少一次 memcpy

Player 发布消息时直接调用 `rcl_publish_serialized_message` 使用 bag 内的序列化 buffer，跳过原来构造 `rclcpp::SerializedMessage` 的一次内存拷贝。大消息回放时降低 CPU 占用。纯内部优化，无 flag、对回放行为无可见变化。

### 4. YAML 配置录制

用 YAML 文件定义要录的 topic 列表及每个 topic 的 QoS profile。

| CLI flag | 说明 |
|---|---|
| `--record-config <yaml>` | 从 YAML 加载 topic 列表 + 每 topic QoS 覆盖 |

YAML 格式（顶层只识别 `topics` 键，值为 `topic_name -> qos_profile` 映射）：
```yaml
topics:
  /chatter:
    reliability: reliable
    durability: volatile
    history: keep_last
    depth: 10
  /odom:
    reliability: best_effort
    depth: 5
  /image_raw:    # QoS 为空表示仅加入录制列表，不设 QoS override
```

```bash
ros2 bag record -o /tmp/bag --record-config /tmp/cfg.yaml
# 也可和命令行 topic 混用（命令行 topic 在前，去重合并）：
ros2 bag record -o /tmp/bag --record-config /tmp/cfg.yaml /extra_topic
```

> `--record-config` 不能与 `-a/--all` 同时使用。对重叠 topic，`--record-config` 的 QoS 优先级高于 `--qos-profile-overrides-path`。此 flag 只配 topics + QoS，不配 storage options。

### 5. trosbag 命令入口

提供独立的 `ros2 tros_bag` 命令入口（`ros2 tros_bag record` / `ros2 tros_bag play`），作为本分支优化版的统一入口，方便后续在 TROS 环境固化优化默认参数。当前行为与 `ros2 bag record` / `ros2 bag play` 完全一致。

```bash
# 先编译并 source
colcon build --packages-select trosbag
source install/setup.bash

ros2 tros_bag record -a -o /tmp/bag
ros2 tros_bag record -o /tmp/bag --no-delay --delay-timeout-ms 100 /chatter
ros2 tros_bag record -o /tmp/bag --record-config /tmp/cfg.yaml
ros2 tros_bag play /tmp/bag
```

### 6. Composable Recorder / Player

`Recorder` 和 `Player` 支持作为 `rclcpp_components` 组件加载到 `component_container_mt`，可常驻运行、与其它节点共进程、由 launch 统一编排。

```bash
# launch 方式（推荐）
ros2 launch trosbag composable_recorder.launch.py bag_uri:=/tmp/bag
ros2 launch trosbag composable_player.launch.py bag_uri:=/tmp/bag

# 手动加载组件（注意：用 Ctrl+C 退容器，不要用 ros2 component unload）
ros2 run rclcpp_components component_container_mt --ros-args -r __node:=rec_ctr
ros2 component load /rec_ctr rosbag2_transport rosbag2_transport::Recorder \
  -p uri:=/tmp/bag -p storage_id:=sqlite3 -p topics:=[/chatter]
```

**已知限制**：
- `ros2 component unload` 在 humble `component_container` 上有卸载时序缺陷（非 rosbag2 bug），建议用 launch 的 Ctrl+C 退出整容器。
- `use_sim_time` 需在组件加载时通过 `-p use_sim_time:=true` 传入（rclcpp humble 的 TimeSource 在构造时同步读取，不能事后 `ros2 param set`）。
- Player standalone 可执行在 ARM cyclonedds 下 SIGINT 不退出，用 launch（容器方式）规避。

## Installation instructions

## Debian packages

rosbag2 packages are available via debian packages and thus can be installed via

```
$ export CHOOSE_ROS_DISTRO=crystal # rosbag2 is available starting from crystal
$ sudo apt-get install ros-$CHOOSE_ROS_DISTRO-ros2bag ros-$CHOOSE_ROS_DISTRO-rosbag2*
```

Note that the above command installs all packages related to rosbag2.
This also includes the plugin for [reading ROS1 bag files](https://github.com/ros2/rosbag2_bag_v2), which brings a hard dependency on the [ros1_bridge](https://github.com/ros2/ros1_bridge) with it and therefore ROS1 packages.
If you want to install only the ROS2 related packages for rosbag, please use the following command:

```
$ export CHOOSE_ROS_DISTRO=crystal # rosbag2 is available starting from crystal
$ sudo apt-get install ros-$CHOOSE_ROS_DISTRO-ros2bag ros-$CHOOSE_ROS_DISTRO-rosbag2-transport
```

## Build from source

It is recommended to create a new overlay workspace on top of your current ROS 2 installation.

```
$ mkdir -p ~/rosbag_ws/src
$ cd ~/rosbag_ws/src
```

Clone this repository into the source folder:

```
$ git clone https://github.com/ros2/rosbag2.git
```
**[Note]**: if you are only building rosbag2 on top of a Debian Installation of ROS2, please git clone the branch following your current ROS2 distribution.

Then build all the packages with this command:

```
$ colcon build [--merge-install]
```

The `--merge-install` flag is optional and installs all packages into one folder rather than isolated folders for each package.

#### Executing tests

The tests can be run using the following commands:

```
$ colcon test [--merge-install]
$ colcon test-result --verbose
```

The first command executes the test and the second command displays the errors (if any).

## Using rosbag2

rosbag2 is part of the ROS 2 command line interfaces.
This repo introduces a new verb called `bag` and thus serves as the entry point of using rosbag2.
As of the time of writing, there are three commands available for `ros2 bag`:

* record
* play
* info

### Recording data

In order to record all topics currently available in the system:

```
$ ros2 bag record -a
```

The command above will record all available topics and discovers new topics as they appear while recording.
This auto-discovery of new topics can be disabled by given the command line argument `--no-discovery`.

To record a set of predefined topics, one can specify them on the command line explicitly.

```
$ ros2 bag record <topic1> <topic2> … <topicN>
```

The specified topics don't necessarily have to be present at start time.
The discovery function will automatically recognize if one of the specified topics appeared.
In the same fashion, this auto discovery can be disabled with `--no-discovery`.

If not further specified, `ros2 bag record` will create a new folder named to the current time stamp and stores all data within this folder.
A user defined name can be given with `-o, --output`.

#### Simulation time

In ROS 2, "simulation time" refers to publishing a clock value on the `/clock` topic, instead of using the system clock to tell time.
By passing `--use-sim-time` argument to `ros2 bag record`, we turn on this option for the recording node.
Messages written to the bag will use the latest received value of `/clock` for the timestamp of the recorded message.

Note: Until the first `/clock` message is received, the recorder will not write any messages.
Before that message is received, the time is 0, which leads to a significant time jump once simulation time begins, making the bag essentially unplayable if messages are written first with time 0 and then time N from `/clock`.

#### Splitting recorded bag files

rosbag2 offers the capability to split bag files when they reach a maximum size or after a specified duration. By default rosbag2 will record all data into a single bag file, but this can be changed using the CLI options.

_Splitting by size_: `ros2 bag record -a -b 100000` will split the bag files when they become greater than 100 kilobytes. Note: the batch size's units are in bytes and must be greater than `86016`. This option defaults to `0`, which means data is written to a single file.

_Splitting by time_: `ros2 bag record -a -d 9000` will split the bag files after a duration of `9000` seconds. This option defaults to `0`, which means data is written to a single file.

If both splitting by size and duration are enabled, the bag will split at whichever threshold is reached first.

#### Recording with compression

By default rosbag2 does not record with compression enabled. However, compression can be specified using the following CLI options.

For example, `ros2 bag record -a --compression-mode file --compression-format zstd` will record all topics and compress each file using the [zstd](https://github.com/facebook/zstd) compressor.

Currently, the only `compression-format` available is `zstd`. Both the mode and format options default to `none`. To use a compression format, a compression mode must be specified, where the currently supported modes are compress by `file` or compress by `message`.

It is recommended to use this feature with the splitting options.

#### Recording with a storage configuration

Storage configuration can be specified in a YAML file passed through the `--storage-config-file` option.
This can be used to optimize performance for specific use-cases.

For the default storage plugin (sqlite3), the file has a following syntax:
```
read:
  pragmas: <list of pragma settings for read-only>
write:
  pragmas: <list of pragma settings for read/write>
```

By default, SQLite settings are significantly optimized for performance.
This might have consequences of bag data being corrupted after an application or system-level crash.
This consideration only applies to current bagfile in case bag splitting is on (through `--max-bag-*` parameters).
If increased crash-caused corruption resistance is necessary, use `resilient` option for `--storage-preset-profile` setting.

Settings are fully exposed to the user and should be applied with understanding.
Please refer to [documentation of pragmas](https://www.sqlite.org/pragma.html).

An example configuration file could look like this:

```
write:
  pragmas: ["journal_mode = MEMORY", "synchronous = OFF", "schema.cache_size = 1000", "schema.page_size = 4096"]

```

### Replaying data

After recording data, the next logical step is to replay this data:

```
$ ros2 bag play <bag_file>
```

The bag file is by default set to the folder name where the data was previously recorded in.

#### Controlling playback via services

The Rosbag2 player provides the following services for remote control, which can be called via `ros2 service` commandline or from your nodes,

* `~/burst [rosbag2_interfaces/srv/Burst]`
  * Can only be used while player is paused, publishes `num_messages` in order as fast as possible, moving forward the play head.
* `~/get_rate [rosbag2_interfaces/srv/GetRate]`
  * Return the current playback rate.
* `~/is_paused [rosbag2_interfaces/srv/IsPaused]`
  * Return whether playback is paused.
* `~/pause [rosbag2_interfaces/srv/Pause]`
  * Pause playback. Has no effect if already paused.
* `~/play [rosbag2_interfaces/srv/Play]`
  * Play from a starting offset timestamp, either until the end, an ending timestamp or for a set duration. Only works when stopped (not paused).
* `~/play_next [rosbag2_interfaces/srv/PlayNext]`
  * Play a single next message from the bag. Only works while paused.
* `~/resume [rosbag2_interfaces/srv/Resume]`
  * Resume playback if paused.
* `~/seek [rosbag2_interfaces/srv/Seek]`
  * Change the play head to the specified timestamp. Can be forward or backward in time, the next played message is the next immediately after the seeked timestamp.
* `~/set_rate [rosbag2_interfaces/srv/SetRate]`
  * Sets the rate of playback, for example 2.0 will play messages twice as fast.
* `~/toggle_paused [rosbag2_interfaces/srv/TogglePaused]`
  * Pause if playing, resume if paused.

### Analyzing data

The recorded data can be analyzed by displaying some meta information about it:

```
$ ros2 bag info <bag_file>
```

You should see something along these lines:

```
Files:             demo_strings.db3
Bag size:          44.5 KiB
Storage id:        sqlite3
Duration:          8.501s
Start:             Nov 28 2018 18:02:18.600 (1543456938.600)
End                Nov 28 2018 18:02:27.102 (1543456947.102)
Messages:          27
Topic information: Topic: /chatter | Type: std_msgs/String | Count: 9 | Serialization Format: cdr
                   Topic: /my_chatter | Type: std_msgs/String | Count: 18 | Serialization Format: cdr
```

### Converting bags

Rosbag2 provides a tool `ros2 bag convert` (or, `rosbag2_transport::bag_rewrite` in the C++ API).
This allows the user to take one or more input bags, and write them out to one or more output bags with new settings.
This flexible feature enables the following features:
* Merge (multiple input bags, one output bag)
* Split top-level bags (one input bag, multiple output bags)
* Split internal files (by time or size - one input bag with fewer internal files, one output bag with more, smaller, internal files)
* Compress/Decompress (output bag(s) with different compression settings than the input(s))
* Serialization format conversion
* ... and more!

Here is an example command:

```
ros2 bag convert --input /path/to/bag1 --input /path/to/bag2 storage_id --output-options output_options.yaml
```

The `--input` argument may be specified any number of times, and takes 1 or 2 values.
The first value is the URI of the input bag.
If a second value is supplied, it specifies the storage implementation of the bag.
If no storage implementation is specified, rosbag2 will try to determine it automatically from the bag.

The `--output-options` argument must point to the URI of a YAML file specifying the full recording configuration for each bag to output (`StorageOptions` + `RecordOptions`).
This file must contain a top-level key `output_bags`, which contains a list of these objects.

The only required value in the output bags is `uri` and `storage_id`. All other values are options (however, if no topic selection is specified, this output bag will be empty!).

This example notes all fields that can have an effect, with a comment on the required ones.

```
output_bags
- uri: /output/bag1  # required
  storage_id: sqlite3  # required
  max_bagfile_size: 0
  max_bagfile_duration: 0
  storage_preset_profile: ""
  storage_config_uri: ""
  all: false
  topics: []
  rmw_serialization_format: ""  # defaults to using the format of the input topic
  regex: ""
  exclude: ""
  compression_mode: ""
  compression_format: ""
  compression_queue_size: 1
  compression_threads: 0
  include_hidden_topics: false
  include_unpublished_topics: false
```

Example merge:

```
$ ros2 bag convert -i bag1 -i bag2 -o out.yaml

# out.yaml
output_bags:
- uri: merged_bag
  storage_id: sqlite3
  all: true
```

Example split:

```
$ ros2 bag convert -i bag1 -o out.yaml

# out.yaml
output_bags:
- uri: split1
  storage_id: sqlite3
  topics: [/topic1, /topic2]
- uri: split2
  storage_id: sqlite3
  topics: [/topic3]
```

Example compress:

```
$ ros2 bag convert -i bag1 -o out.yaml

# out.yaml
output_bags:
- uri: compressed
  storage_id: sqlite3
  all: true
  compression_mode: file
  compression_format: zstd
```

### Overriding QoS Profiles

When starting a recording or playback workflow, you can pass a YAML file that contains QoS profile settings for a specific topic.
The YAML schema for the profile overrides is a dictionary of topic names with key/value pairs for each QoS policy.
Below is an example profile set to the default ROS 2 QoS settings.

```yaml
/topic_name:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: volatile
  deadline:
    # unspecified/infinity
    sec: 0
    nsec: 0
  lifespan:
    # unspecified/infinity
    sec: 0
    nsec: 0
  liveliness: system_default
  liveliness_lease_duration:
    # unspecified/infinity
    sec: 0
    nsec: 0
  avoid_ros_namespace_conventions: false
```

You can then use the override by specifying the `--qos-profile-overrides-path` argument in the CLI:

```sh
# Record
ros2 bag record --qos-profile-overrides-path override.yaml -a -o my_bag
# Playback
ros2 bag play --qos-profile-overrides-path override.yaml my_bag
```

See [the official QoS override tutorial][qos-override-tutorial] and ["About QoS Settings"][about-qos-settings] for more detail.

### Using in launch

We can invoke the command line tool from a ROS launch script as an *executable* (not a *node* action).
For example, to launch the command to record all topics you can use the following launch script:

```xml
<launch>
  <executable cmd="ros2 bag record -a" output="screen" />
</launch>
```

Here's the equivalent Python launch script:

```python
import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        )
    ])
```

Use the `ros2 launch` command line tool to launch either of the above launch scripts.
For example, if we named the above XML launch script, `record_all.launch.xml`:

```sh
$ ros2 launch record_all.launch.xml
```

## Storage format plugin architecture

Looking at the output of the `ros2 bag info` command, we can see a field called `storage id:`.
rosbag2 specifically was designed to support multiple storage formats.
This allows a flexible adaptation of various storage formats depending on individual use cases.
As of now, this repository comes with two storage plugins.
The first plugin, sqlite3 is chosen by default.
If not specified otherwise, rosbag2 will store and replay all recorded data in an SQLite3 database.

In order to use a specified (non-default) storage format plugin, rosbag2 has a command line argument for it:

```
$ ros2 bag <record> | <play> | <info> -s <sqlite3> | <rosbag2_v2> | <custom_plugin>
```

Have a look at each of the individual plugins for further information.

## Serialization format plugin architecture

Looking further at the output of `ros2 bag info`, we can see another field attached to each topic called `Serialization Format`.
By design, ROS 2 is middleware agnostic and thus can leverage multiple communication frameworks.
The default middleware for ROS 2 is DDS which has `cdr` as its default binary serialization format.
However, other middleware implementation might have different formats.
If not specified, `ros2 bag record -a` will record all data in the middleware specific format.
This however also means that such a bag file can't easily be replayed with another middleware format.

rosbag2 implements a serialization format plugin architecture which allows the user the specify a certain serialization format.
When specified, rosbag2 looks for a suitable converter to transform the native middleware protocol to the target format.
This also allows to record data in a native format to optimize for speed, but to convert or transform the recorded data into a middleware agnostic serialization format.

By default, rosbag2 can convert from and to CDR as it's the default serialization format for ROS 2.

[qos-override-tutorial]: https://docs.ros.org/en/rolling/Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html
[about-qos-settings]: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html

