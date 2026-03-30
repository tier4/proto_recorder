// Copyright 2024 TIER IV, Inc.
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

#include "proto_recorder/proto_recorder.hpp"

#include <proto_recorder_msgs/msg/recorder_status.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

class NodeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(NodeIntegrationTest, NodeConstruction_DefaultParams)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("start_recording", false);
  EXPECT_NO_THROW(std::make_shared<proto_recorder::ProtoRecorder>(options));
}

TEST_F(NodeIntegrationTest, StatusPublished_Periodically)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("start_recording", false);
  options.append_parameter_override("diagnostics_period", 0.1);
  auto node = std::make_shared<proto_recorder::ProtoRecorder>(options);

  // Create a subscriber to receive status messages
  proto_recorder_msgs::msg::RecorderStatus::SharedPtr received_msg;
  auto sub_node = std::make_shared<rclcpp::Node>("test_subscriber");
  auto subscription = sub_node->create_subscription<proto_recorder_msgs::msg::RecorderStatus>(
    "/proto_recorder/output/status", rclcpp::QoS(10),
    [&received_msg](proto_recorder_msgs::msg::RecorderStatus::SharedPtr msg) {
      received_msg = msg;
    });

  // Spin until we receive a message or timeout
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->add_node(sub_node);

  auto start = std::chrono::steady_clock::now();
  while (!received_msg && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(5)) {
    executor->spin_some(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received_msg, nullptr) << "Did not receive status message within timeout";
  EXPECT_FALSE(received_msg->hardware_id.empty());
  EXPECT_FALSE(received_msg->is_recording);
  EXPECT_EQ(received_msg->error_level, proto_recorder_msgs::msg::RecorderStatus::ERROR_LEVEL_OK);
}

TEST_F(NodeIntegrationTest, StartViaTopicCommand)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("start_recording", false);
  options.append_parameter_override("uri", "/tmp/proto_recorder_test_start_cmd");
  options.append_parameter_override("diagnostics_period", 0.1);
  auto node = std::make_shared<proto_recorder::ProtoRecorder>(options);

  auto pub_node = std::make_shared<rclcpp::Node>("test_publisher");
  auto start_pub = pub_node->create_publisher<std_msgs::msg::Bool>(
    "/proto_recorder/input/start", rclcpp::QoS(1).transient_local());

  proto_recorder_msgs::msg::RecorderStatus::SharedPtr status_msg;
  auto sub_node = std::make_shared<rclcpp::Node>("test_status_sub");
  auto status_sub = sub_node->create_subscription<proto_recorder_msgs::msg::RecorderStatus>(
    "/proto_recorder/output/status", rclcpp::QoS(10),
    [&status_msg](proto_recorder_msgs::msg::RecorderStatus::SharedPtr msg) { status_msg = msg; });

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->add_node(pub_node);
  executor->add_node(sub_node);

  // Send start command
  auto msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = true;
  start_pub->publish(std::move(msg));

  // Wait for status to show recording
  auto start_time = std::chrono::steady_clock::now();
  bool found_recording = false;
  while ((std::chrono::steady_clock::now() - start_time) < std::chrono::seconds(5)) {
    executor->spin_some(std::chrono::milliseconds(10));
    if (status_msg && status_msg->is_recording) {
      found_recording = true;
      break;
    }
  }

  EXPECT_TRUE(found_recording) << "Recording did not start via topic command";

  // Cleanup: stop recording and remove temp bag
  auto stop_msg = std::make_unique<std_msgs::msg::Bool>();
  stop_msg->data = false;
  start_pub->publish(std::move(stop_msg));
  executor->spin_some(std::chrono::milliseconds(100));
}
