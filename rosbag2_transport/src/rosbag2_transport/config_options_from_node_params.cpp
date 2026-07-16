// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosbag2_transport/config_options_from_node_params.hpp"

#include <cctype>
#include <chrono>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"

#include "rcutils/time.h"

#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/qos.hpp"
#include "rosbag2_transport/record_options.hpp"

namespace
{
// Validate that a parameter read as a signed int64 is non-negative before it is assigned to an
// unsigned struct field (uint64_t / size_t). A negative value would otherwise silently wrap to
// a huge unsigned value; throwing here surfaces it at component-load time with a clear message,
// matching how `ros2 bag record`/`play` reject negatives.
int64_t require_non_negative(const rclcpp::Node & node, const std::string & name, int64_t value)
{
  if (value < 0) {
    throw std::invalid_argument(
            "Parameter '" + name + "' must be non-negative, got " + std::to_string(value) + ".");
  }
  (void)node;
  return value;
}

// Parse a QoS reliability/durability/history/liveliness policy from a (case-insensitive) string
// name, matching the format accepted by the `ros2 bag record --qos-profile-overrides-path` CLI
// option (e.g. "reliable", "transient_local"). Returns false if the name is not recognized.
bool policy_from_string(const std::string & s, rmw_qos_reliability_policy_t & out)
{
  std::string lower;
  lower.reserve(s.size());
  for (auto c : s) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (lower == "reliable") {out = RMW_QOS_POLICY_RELIABILITY_RELIABLE; return true;}
  if (lower == "best_effort" || lower == "best-effort") {
    out = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    return true;
  }
  return false;
}

bool policy_from_string(const std::string & s, rmw_qos_durability_policy_t & out)
{
  std::string lower;
  lower.reserve(s.size());
  for (auto c : s) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (lower == "transient_local" || lower == "transient-local") {
    out = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    return true;
  }
  if (lower == "volatile") {out = RMW_QOS_POLICY_DURABILITY_VOLATILE; return true;}
  return false;
}

bool policy_from_string(const std::string & s, rmw_qos_history_policy_t & out)
{
  std::string lower;
  lower.reserve(s.size());
  for (auto c : s) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (lower == "keep_last" || lower == "keep-last") {
    out = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    return true;
  }
  if (lower == "keep_all" || lower == "keep-all") {
    out = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    return true;
  }
  return false;
}

// Build a rclcpp::QoS from a yaml node that uses the same string-based policy names as the
// `--qos-profile-overrides-path` CLI file (see ros2bag.api.convert_yaml_to_qos_profile).
rclcpp::QoS qos_from_yaml_node(const YAML::Node & profile)
{
  rclcpp::QoS qos(rmw_qos_profile_default.depth);

  // Set depth BEFORE history: rclcpp::QoS::keep_last(depth) also forces history to KEEP_LAST,
  // so if the user asked for history: keep_all we must not let a later keep_last() clobber it.
  // Setting history() after keep_last() leaves depth intact and honors the requested policy.
  if (profile["depth"]) {
    qos.keep_last(profile["depth"].as<size_t>());
  }
  if (profile["history"]) {
    rmw_qos_history_policy_t h{};
    if (policy_from_string(profile["history"].as<std::string>(), h)) {
      qos.history(h);
    }
  }
  if (profile["reliability"]) {
    rmw_qos_reliability_policy_t r{};
    if (policy_from_string(profile["reliability"].as<std::string>(), r)) {
      qos.reliability(r);
    }
  }
  if (profile["durability"]) {
    rmw_qos_durability_policy_t d{};
    if (policy_from_string(profile["durability"].as<std::string>(), d)) {
      qos.durability(d);
    }
  }
  return qos;
}

// Load per-topic QoS overrides from a yaml file whose top-level keys are topic names and whose
// values are QoS profile mappings (same format as `--qos-profile-overrides-path`).
// On any parse error, logs a clear message and returns empty overrides so a malformed file
// never crashes component load.
std::unordered_map<std::string, rclcpp::QoS>
load_qos_profile_overrides(const std::string & path)
{
  std::unordered_map<std::string, rclcpp::QoS> overrides;
  if (path.empty() || !std::filesystem::exists(path)) {
    return overrides;
  }
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root.IsMap()) {
      throw std::runtime_error("QoS overrides file is not a YAML mapping");
    }
    for (const auto & kv : root) {
      const std::string topic = kv.first.as<std::string>();
      overrides.emplace(topic, qos_from_yaml_node(kv.second));
    }
  } catch (const std::exception & e) {
    std::cerr << "[rosbag2_transport] Failed to parse QoS overrides file '" << path
              << "': " << e.what() << " (ignoring overrides)" << std::endl;
    overrides.clear();
  }
  return overrides;
}
}  // namespace

namespace rosbag2_transport
{

rosbag2_storage::StorageOptions
get_storage_options_from_node_params(rclcpp::Node & node)
{
  rosbag2_storage::StorageOptions storage_options;

  storage_options.uri = node.declare_parameter<std::string>("uri", "");
  storage_options.storage_id = node.declare_parameter<std::string>("storage_id", "");
  // An empty 'uri' is the most common component-load mistake (a missing bag path), but the
  // underlying storage plugin reports it as an opaque 'Failed to create database directory ()'
  // or 'No storage could be initialized'. Reject it up front with a clear message instead.
  if (storage_options.uri.empty()) {
    throw std::invalid_argument(
            "Parameter 'uri' must be set to the bag path when running as a component.");
  }
  storage_options.max_bagfile_size =
    require_non_negative(node, "max_bagfile_size",
      node.declare_parameter<int64_t>("max_bagfile_size", 0));
  storage_options.max_bagfile_duration =
    require_non_negative(node, "max_bagfile_duration",
      node.declare_parameter<int64_t>("max_bagfile_duration", 0));
  storage_options.max_cache_size =
    require_non_negative(node, "max_cache_size",
      node.declare_parameter<int64_t>("max_cache_size", 100 * 1024 * 1024));
  storage_options.storage_preset_profile =
    node.declare_parameter<std::string>("storage_preset_profile", "");
  storage_options.storage_config_uri =
    node.declare_parameter<std::string>("storage_config_uri", "");
  storage_options.snapshot_mode =
    node.declare_parameter<bool>("snapshot_mode", false);
  // d-robotics perf extension: delayed cache consumption.
  // Accept as int64 so YAML `delay: 0` (integer) does not cause a type-mismatch at load time.
  // Any non-zero value is treated as true; 0 is false.
  storage_options.delay =
    (node.declare_parameter<int64_t>("delay", 1) != 0);
  storage_options.delay_timeout_ms =
    require_non_negative(node, "delay_timeout_ms",
      node.declare_parameter<int64_t>("delay_timeout_ms", 200));

  return storage_options;
}

rosbag2_transport::RecordOptions
get_record_options_from_node_params(rclcpp::Node & node)
{
  rosbag2_transport::RecordOptions record_options;

  record_options.all = node.declare_parameter<bool>("all", false);
  record_options.is_discovery_disabled =
    node.declare_parameter<bool>("is_discovery_disabled", false);
  record_options.topics =
    node.declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
  record_options.rmw_serialization_format =
    node.declare_parameter<std::string>("rmw_serialization_format", "");
  record_options.topic_polling_interval = std::chrono::milliseconds(
    require_non_negative(node, "topic_polling_interval",
      node.declare_parameter<int64_t>("topic_polling_interval", 100)));
  record_options.regex = node.declare_parameter<std::string>("regex", "");
  record_options.exclude = node.declare_parameter<std::string>("exclude", "");
  record_options.node_prefix = node.declare_parameter<std::string>("node_prefix", "");
  record_options.compression_mode =
    node.declare_parameter<std::string>("compression_mode", "");
  record_options.compression_format =
    node.declare_parameter<std::string>("compression_format", "");
  record_options.compression_queue_size =
    require_non_negative(node, "compression_queue_size",
      node.declare_parameter<int64_t>("compression_queue_size", 1));
  record_options.compression_threads =
    require_non_negative(node, "compression_threads",
      node.declare_parameter<int64_t>("compression_threads", 0));
  record_options.include_hidden_topics =
    node.declare_parameter<bool>("include_hidden_topics", false);
  record_options.include_unpublished_topics =
    node.declare_parameter<bool>("include_unpublished_topics", false);
  record_options.ignore_leaf_topics =
    node.declare_parameter<bool>("ignore_leaf_topics", false);
  record_options.start_paused =
    node.declare_parameter<bool>("start_paused", false);
  // `use_sim_time` is a standard ROS parameter that rclcpp/the container may already have
  // declared, so read it instead of declaring it ourselves.
  record_options.use_sim_time = node.get_parameter_or("use_sim_time", false);

  // Per-topic QoS overrides are loaded from a yaml file (same format as the
  // `--qos-profile-overrides-path` CLI option) referenced by the `qos_profile_overrides_path`
  // parameter.
  const std::string qos_overrides_path =
    node.declare_parameter<std::string>("qos_profile_overrides_path", "");
  record_options.topic_qos_profile_overrides = load_qos_profile_overrides(qos_overrides_path);

  return record_options;
}

rosbag2_transport::PlayOptions
get_play_options_from_node_params(rclcpp::Node & node)
{
  rosbag2_transport::PlayOptions play_options;

  play_options.read_ahead_queue_size =
    require_non_negative(node, "read_ahead_queue_size",
      node.declare_parameter<int64_t>("read_ahead_queue_size", 1000));
  play_options.node_prefix = node.declare_parameter<std::string>("node_prefix", "");
  play_options.rate =
    static_cast<float>(node.declare_parameter<double>("rate", 1.0));
  play_options.topics_to_filter =
    node.declare_parameter<std::vector<std::string>>("topics_to_filter",
      std::vector<std::string>{});
  play_options.loop = node.declare_parameter<bool>("loop", false);
  // topic_remapping_options: when using the component, supply remappings via the standard
  // `--ros-args -r` mechanism (handled by the container), not via this parameter.
  play_options.topic_remapping_options = {};
  play_options.clock_publish_frequency =
    node.declare_parameter<double>("clock_publish_frequency", 0.0);
  // delay is a rclcpp::Duration (seconds + nanoseconds); accept it in seconds (float).
  // Named 'playback_delay' (not 'delay') to avoid clashing with the StorageOptions 'delay'
  // parameter (the d-robotics record-side cache optimization) declared above, since both
  // storage and play options are loaded onto the same Player node.
  const double delay_s = node.declare_parameter<double>("playback_delay", 0.0);
  if (delay_s < 0.0) {
    throw std::invalid_argument(
            "Parameter 'playback_delay' must be non-negative, got " +
            std::to_string(delay_s) + " s.");
  } else {
    play_options.delay = rclcpp::Duration::from_seconds(delay_s);
  }
  play_options.start_paused = node.declare_parameter<bool>("start_paused", false);
  // start_offset is a rcutils_time_point_value_t (int64 nanoseconds). The CLI `--start-offset`
  // accepts a value in SECONDS (ros2bag play.py parses a float; the pybind path applies
  // RCUTILS_S_TO_NS), so declare this parameter as seconds (double) here too and convert, so
  // the composable path matches `ros2 bag play --start-offset N` rather than differing by 1e9x.
  const double start_offset_s = node.declare_parameter<double>("start_offset", 0.0);
  if (start_offset_s < 0.0) {
    RCLCPP_WARN(
      node.get_logger(),
      "Negative 'start_offset' parameter (%.6f s) is invalid; ignoring.", start_offset_s);
    play_options.start_offset = 0;
  } else {
    play_options.start_offset = RCUTILS_S_TO_NS(start_offset_s);
  }
  play_options.disable_keyboard_controls =
    node.declare_parameter<bool>("disable_keyboard_controls", false);
  play_options.wait_acked_timeout =
    node.declare_parameter<int64_t>("wait_acked_timeout", -1);
  play_options.disable_loan_message =
    node.declare_parameter<bool>("disable_loan_message", false);

  // Per-topic QoS overrides (same yaml format as `ros2 bag play --qos-profile-overrides-path`).
  const std::string qos_overrides_path =
    node.declare_parameter<std::string>("qos_profile_overrides_path", "");
  play_options.topic_qos_profile_overrides = load_qos_profile_overrides(qos_overrides_path);

  return play_options;
}

}  // namespace rosbag2_transport
