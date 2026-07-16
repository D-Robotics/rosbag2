// Copyright 2023 Patrick Roncagliolo and Michael Orlov
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

#include <gmock/gmock.h>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "rosbag2_transport/recorder.hpp"

class ComposableRecorderTestFixture : public Rosbag2TransportTestFixture
{
public:
  ComposableRecorderTestFixture()
  : Rosbag2TransportTestFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~ComposableRecorderTestFixture() override
  {
    rclcpp::shutdown();
  }

  // A unique temporary directory for each test fixture instance, cleaned up on destruction.
  static std::string make_temp_bag_uri()
  {
    auto dir = std::filesystem::temp_directory_path() / "rosbag2_composable_recorder_test";
    // Make sure the directory does not exist so the recorder creates a fresh bag.
    std::filesystem::remove_all(dir);
    return dir.string();
  }
};

class ComposableRecorder : public rosbag2_transport::Recorder
{
public:
  static const char demo_attribute_name_[];
  bool demo_attribute_value{false};

  explicit ComposableRecorder(const rclcpp::NodeOptions & options)
  : rosbag2_transport::Recorder(
      std::make_shared<rosbag2_cpp::Writer>(),
      rosbag2_storage::StorageOptions(),
      rosbag2_transport::RecordOptions(),
      "test_recorder_component",
      options)
  {
    // Declare demo attribute parameter for the underlying node with default value equal to false.
    // However, if node was created with option to override this parameter it will be settled up
    // to what was specified in parameter_overrides value.
    demo_attribute_value = this->declare_parameter<bool>(
      demo_attribute_name_, /*default_value=*/ false,
      rcl_interfaces::msg::ParameterDescriptor(), /*ignore_override=*/ false);
  }

  bool get_value_of_bool_parameter(const std::string & parameter_name)
  {
    bool ret_value{false};
    bool parameter_was_set = this->get_parameter(parameter_name, ret_value);
    if (!parameter_was_set) {
      throw std::runtime_error("Parameter `" + parameter_name + "` hasn't been set.");
    }
    return ret_value;
  }
};
const char ComposableRecorder::demo_attribute_name_[] = "demo_attribute";

TEST_F(ComposableRecorderTestFixture, recorder_inner_params_passed_as_append_override)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back(ComposableRecorder::demo_attribute_name_, true);
  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto recorder = std::make_shared<ComposableRecorder>(options);
  // Check that rosbag2_transport::Recorder inner params will not erase our
  // parameter_overrides options
  ASSERT_TRUE(recorder->get_value_of_bool_parameter(recorder->demo_attribute_name_));
  ASSERT_TRUE(recorder->demo_attribute_value);
}

// Verify that the default (composable) constructor no longer throws UnimplementedError and that
// it loads storage/record options from node parameters, starts recording, and can be cleanly
// stopped - producing a bag directory on disk.
TEST_F(ComposableRecorderTestFixture, default_ctor_loads_options_from_node_params)
{
  const std::string uri = make_temp_bag_uri();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  // Disable discovery so the recorder won't keep polling for new topics, and give it a topic
  // that does not exist so subscribe_topics() finds nothing to subscribe to - this keeps the
  // test focused on the construction/parameter-loading/stop path without real data flow.
  // (The composable ctor now validates that at least one of all/topics/regex is set, matching
  // the `ros2 bag record` CLI behavior.)
  parameters.emplace_back("is_discovery_disabled", true);
  parameters.emplace_back("all", false);
  parameters.emplace_back("topics", std::vector<std::string>{"/nonexistent_test_topic"});
  parameters.emplace_back("start_paused", true);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  // The default constructor delegates to the parameter-loading path and calls record().
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      "test_recorder_component_default", options);

  // Sanity-check that the storage uri parameter was parsed into the recorder's options.
  ASSERT_EQ(recorder->get_storage_options().uri, uri);
  ASSERT_TRUE(recorder->get_storage_options().delay);  // d-robotics perf extension default
  ASSERT_TRUE(recorder->get_record_options().start_paused);
  ASSERT_TRUE(recorder->get_record_options().is_discovery_disabled);
  ASSERT_EQ(
    recorder->get_record_options().topics,
    std::vector<std::string>({"/nonexistent_test_topic"}));

  // Stopping must be safe even when discovery was disabled and no topics were subscribed.
  EXPECT_NO_THROW(recorder->stop());

  // The writer should have created the bag directory on disk.
  EXPECT_TRUE(std::filesystem::exists(uri));

  recorder.reset();
  std::filesystem::remove_all(uri);
}

// Bug regression: a missing/empty 'uri' must be rejected up front with a clear message instead of
// producing an opaque 'Failed to create database directory ()' from writer->open().
TEST_F(ComposableRecorderTestFixture, empty_uri_is_rejected_up_front)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("all", true);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Recorder>("test_recorder_no_uri", options),
    std::invalid_argument);
}

// Bug regression: a negative max_cache_size (declared int64, field uint64) must throw at load
// instead of silently wrapping to ~1.8e19.
TEST_F(ComposableRecorderTestFixture, negative_max_cache_size_is_rejected)
{
  const std::string uri = make_temp_bag_uri();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("all", true);
  parameters.emplace_back("max_cache_size", int64_t(-1));

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Recorder>("test_recorder_neg_cache", options),
    std::invalid_argument);

  std::filesystem::remove_all(uri);
}

// Bug regression: 'all' + 'topics' is mutually exclusive; the composable ctor must reject it
// (matching `ros2 bag record`), not silently accept it.
TEST_F(ComposableRecorderTestFixture, all_and_topics_are_mutually_exclusive)
{
  const std::string uri = make_temp_bag_uri();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("all", true);
  parameters.emplace_back("topics", std::vector<std::string>{"/chatter"});

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Recorder>("test_recorder_all_topics", options),
    std::invalid_argument);

  std::filesystem::remove_all(uri);
}

// Bug regression: compression_format set without a compression_mode must be rejected up front
// (matching `ros2 bag record` behavior), not silently produce an uncompressed bag.
TEST_F(ComposableRecorderTestFixture, compression_format_without_mode_is_rejected)
{
  const std::string uri = make_temp_bag_uri();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("all", true);
  parameters.emplace_back("compression_format", std::string("zstd"));
  // compression_mode intentionally left at default (empty string == "none")

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Recorder>("test_recorder_fmt_no_mode", options),
    std::invalid_argument);

  std::filesystem::remove_all(uri);
}

// Verify that passing 'delay' as an integer (YAML `delay: 0`) does not cause a
// rclcpp type-mismatch exception at component load time.
TEST_F(ComposableRecorderTestFixture, delay_integer_zero_is_accepted_as_false)
{
  const std::string uri = make_temp_bag_uri();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("all", false);
  parameters.emplace_back("topics", std::vector<std::string>{"/nonexistent_delay_test"});
  parameters.emplace_back("is_discovery_disabled", true);
  parameters.emplace_back("start_paused", true);
  // Simulate YAML `delay: 0` (integer, common YAML idiom for false)
  parameters.emplace_back("delay", int64_t(0));

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  std::shared_ptr<rosbag2_transport::Recorder> recorder;
  ASSERT_NO_THROW(
    recorder = std::make_shared<rosbag2_transport::Recorder>("test_recorder_delay_int", options));

  // delay=0 should map to storage_options.delay == false
  EXPECT_FALSE(recorder->get_storage_options().delay);

  recorder->stop();
  recorder.reset();
  std::filesystem::remove_all(uri);
}

// Bug regression: a negative topic_polling_interval must throw at component load (it would
// otherwise cause a busy-loop in the discovery thread).
TEST_F(ComposableRecorderTestFixture, negative_topic_polling_interval_is_rejected)
{
  const std::string uri = make_temp_bag_uri();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("all", true);
  parameters.emplace_back("topic_polling_interval", int64_t(-1));

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Recorder>(
      "test_recorder_neg_polling", options),
    std::invalid_argument);

  std::filesystem::remove_all(uri);
}
