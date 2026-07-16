// Copyright 2023 D-Robotics
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
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_transport/player.hpp"

// End-to-end test for the Player composable (default) constructor: it must no longer throw
// UnimplementedError, must load storage/play options from node parameters, and must actually
// play back messages from the bag.
class ComposablePlayerTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  // Write a tiny bag (a few std_msgs/String messages on /chatter) for the Player to open.
  static std::string make_test_bag()
  {
    const auto dir = std::filesystem::temp_directory_path() / "rosbag2_composable_player_test_bag";
    std::filesystem::remove_all(dir);

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = dir.string();
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::Writer writer;
    writer.open(storage_options, {"cdr", "cdr"});

    rosbag2_storage::TopicMetadata topic;
    topic.name = "/chatter";
    topic.type = "std_msgs/msg/String";
    topic.serialization_format = "cdr";
    writer.create_topic(topic);

    // A minimal CDR-serialized std_msgs/String: 4-byte header + 4-byte length + payload.
    const std::string payload = "hello";
    auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    msg->topic_name = "/chatter";
    // CDR encapsulation header (00 01 00 00) + string length (u32 LE) + chars + NUL + pad.
    std::string data = std::string("\x00\x01\x00\x00", 4) +
      std::string(1, static_cast<char>(payload.size() + 1)) + "\x00\x00\x00" + payload + "\x00";
    msg->serialized_data = std::make_shared<rcl_serialized_message_t>();
    msg->serialized_data->buffer = reinterpret_cast<uint8_t *>(data.data());
    msg->serialized_data->buffer_length = data.size();
    msg->serialized_data->buffer_capacity = data.size();
    msg->time_stamp = 0;
    for (int i = 0; i < 3; ++i) {
      writer.write(msg);
      msg->time_stamp += 1000000;  // 1ms apart
    }
    writer.close();
    return dir.string();
  }
};

// The default (composable) constructor must not throw UnimplementedError and must parse the uri
// parameter into the Player's storage options.
TEST_F(ComposablePlayerTestFixture, default_ctor_loads_options_from_node_params)
{
  const std::string bag_uri = make_test_bag();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", bag_uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("rate", 1.0);
  parameters.emplace_back("disable_keyboard_controls", true);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto player = std::make_shared<rosbag2_transport::Player>("test_player_component", options);

  // Storage uri parsed correctly.
  ASSERT_EQ(player->get_storage_options().uri, bag_uri);
  // Play options parsed correctly.
  ASSERT_DOUBLE_EQ(player->get_play_options().rate, 1.0);
  ASSERT_TRUE(player->get_play_options().disable_keyboard_controls);

  // The composable ctor must return immediately (play() runs on a background thread). Before the
  // fix it blocked on play(); with a 3-message bag the ctor would still return, so this is a
  // weak guard - the strong guard is that reset() below does not deadlock on a joined thread.
  // Let playback run briefly, then destroy (destructor signals stop_playback_ + joins play_thread).
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_NO_THROW(player.reset());

  std::filesystem::remove_all(bag_uri);
}

// Bug regression: the composable ctor must assign play_options_ BEFORE prepare_publishers() so
// that topics_to_filter / clock_publish_frequency actually take effect (previously they were
// silently dropped because prepare_publishers read the default-constructed member).
TEST_F(ComposablePlayerTestFixture, play_options_take_effect_for_publishers)
{
  const std::string bag_uri = make_test_bag();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", bag_uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("disable_keyboard_controls", true);
  parameters.emplace_back("topics_to_filter",
    std::vector<std::string>{"/chatter"});
  parameters.emplace_back("clock_publish_frequency", 50.0);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto player = std::make_shared<rosbag2_transport::Player>("test_player_filter", options);

  // The user-supplied values must be reflected in the parsed play options.
  ASSERT_EQ(player->get_play_options().topics_to_filter,
    std::vector<std::string>({"/chatter"}));
  ASSERT_DOUBLE_EQ(player->get_play_options().clock_publish_frequency, 50.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_NO_THROW(player.reset());

  std::filesystem::remove_all(bag_uri);
}

// Bug regression: start_offset is in SECONDS via the composable path (matching `ros2 bag play
// --start-offset N`), converted to nanoseconds. Previously it was declared as int64 nanoseconds,
// differing from the CLI by 1e9x. Verify the seconds->ns conversion (5.0 s == 5e9 ns).
TEST_F(ComposablePlayerTestFixture, start_offset_is_seconds_and_converts_to_ns)
{
  const std::string bag_uri = make_test_bag();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", bag_uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("disable_keyboard_controls", true);
  parameters.emplace_back("start_offset", 5.0);  // seconds

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto player = std::make_shared<rosbag2_transport::Player>("test_player_offset", options);

  // 5 seconds must become 5,000,000,000 ns, not 5 ns.
  EXPECT_EQ(player->get_play_options().start_offset, 5000000000LL);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_NO_THROW(player.reset());

  std::filesystem::remove_all(bag_uri);
}

// Bug regression: a negative start_offset must be clamped to 0 with a warning, not wrap or throw.
TEST_F(ComposablePlayerTestFixture, negative_start_offset_is_ignored)
{
  const std::string bag_uri = make_test_bag();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", bag_uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("disable_keyboard_controls", true);
  parameters.emplace_back("start_offset", -2.0);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto player = std::make_shared<rosbag2_transport::Player>("test_player_neg_offset", options);
  EXPECT_EQ(player->get_play_options().start_offset, 0);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_NO_THROW(player.reset());

  std::filesystem::remove_all(bag_uri);
}

// Bug regression (critical hang): a composable Player constructed with start_paused:=true used to
// deadlock in ~Player() because the trailing `while (is_paused() && rclcpp::ok())` loop never
// checked stop_playback_, and the destructor did not wake the clock's cv. The fix pauses the clock
// (notifying the cv) and adds stop_playback_ to the loop. Verify reset() returns in bounded time.
TEST_F(ComposablePlayerTestFixture, start_paused_destructor_does_not_hang)
{
  const std::string bag_uri = make_test_bag();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", bag_uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("disable_keyboard_controls", true);
  parameters.emplace_back("start_paused", true);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto player = std::make_shared<rosbag2_transport::Player>("test_player_paused", options);
  ASSERT_TRUE(player->get_play_options().start_paused);
  // Let the play thread enter the paused loop.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Destroy via std::async so we can bound the wait. Before the fix the destructor joined forever
  // (the paused tail loop ignored stop_playback_ and the dtor did not wake the clock cv); after
  // the fix it returns within ~100ms (one paused-sleep tick). 5s is a generous bound for slow CI.
  auto destroy_task = std::async(std::launch::async, [&]() {player.reset();});
  if (destroy_task.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    FAIL() << "Player destructor with start_paused hung (regression: stop_playback_ not honored "
              "in paused tail loop, or dtor did not wake clock cv)";
  }
  SUCCEED();

  std::filesystem::remove_all(bag_uri);
}

// Bug regression: a missing/empty 'uri' must be rejected up front with a clear message instead of
// producing an opaque storage-plugin error deep inside reader->open().
TEST_F(ComposablePlayerTestFixture, empty_uri_is_rejected_up_front)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("disable_keyboard_controls", true);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Player>("test_player_no_uri", options),
    std::invalid_argument);
}

// Bug regression: negative uint64/size_t parameters must throw at component load (they previously
// wrapped silently to huge unsigned values). read_ahead_queue_size is size_t.
TEST_F(ComposablePlayerTestFixture, negative_read_ahead_queue_size_is_rejected)
{
  const std::string bag_uri = make_test_bag();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", bag_uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("disable_keyboard_controls", true);
  parameters.emplace_back("read_ahead_queue_size", int64_t(-1));

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Player>("test_player_neg_qsize", options),
    std::invalid_argument);

  std::filesystem::remove_all(bag_uri);
}

// Bug regression: a negative playback_delay must throw std::invalid_argument (consistent with
// require_non_negative() used for integer parameters; old code only warned and clamped to 0).
TEST_F(ComposablePlayerTestFixture, negative_playback_delay_is_rejected)
{
  const std::string bag_uri = make_test_bag();
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("uri", bag_uri);
  parameters.emplace_back("storage_id", std::string("sqlite3"));
  parameters.emplace_back("disable_keyboard_controls", true);
  parameters.emplace_back("playback_delay", -1.0);

  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Player>("test_player_neg_delay", options),
    std::invalid_argument);

  std::filesystem::remove_all(bag_uri);
}
