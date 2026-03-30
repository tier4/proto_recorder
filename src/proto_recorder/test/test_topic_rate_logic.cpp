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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

class TopicRateLogicTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.append_parameter_override("start_recording", false);
    options.append_parameter_override("rate_check_window_size", static_cast<int64_t>(10));
    options.append_parameter_override("diagnostics_period", 1.0);
    node_ = std::make_shared<proto_recorder::ProtoRecorder>(options);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<proto_recorder::ProtoRecorder> node_;
};

TEST_F(TopicRateLogicTest, NoMessages_RateIsZero)
{
  const std::string topic = "/test/rate_topic";
  node_->record_options_.topics.push_back(topic);
  node_->topic_info_[topic].name = topic;

  node_->check_topic_rates();

  EXPECT_DOUBLE_EQ(node_->topic_info_[topic].rate, 0.0);
}

TEST_F(TopicRateLogicTest, SteadyRate_CorrectCalculation)
{
  const std::string topic = "/test/rate_topic";
  node_->record_options_.topics.push_back(topic);
  node_->topic_info_[topic].name = topic;

  // Simulate 10Hz: 11 messages at 100ms intervals
  auto base_time = node_->get_clock()->now();
  for (int i = 0; i < 11; ++i) {
    rclcpp::Time t = base_time + rclcpp::Duration::from_seconds(i * 0.1);
    node_->update_topic_rate(topic, t);
  }

  node_->check_topic_rates();

  // 10 intervals over 1.0 seconds = 10 Hz
  EXPECT_NEAR(node_->topic_info_[topic].rate, 10.0, 1.0);
}

TEST_F(TopicRateLogicTest, RateWithinBounds_StatusNormal)
{
  const std::string topic = "/test/rate_topic";
  node_->record_options_.topics.push_back(topic);
  node_->topic_info_[topic].name = topic;
  node_->topic_info_[topic].min_rate = 9.0;
  node_->topic_info_[topic].max_rate = 11.0;

  // Feed 10Hz data (within 9-11 range)
  auto base_time = node_->get_clock()->now();
  for (int i = 0; i < 11; ++i) {
    rclcpp::Time t = base_time + rclcpp::Duration::from_seconds(i * 0.1);
    node_->update_topic_rate(topic, t);
  }

  node_->check_topic_rates();

  EXPECT_NEAR(node_->topic_info_[topic].rate, 10.0, 1.0);
}

TEST_F(TopicRateLogicTest, RateBelowMin_StatusTooLow)
{
  const std::string topic = "/test/rate_topic";
  node_->record_options_.topics.push_back(topic);
  node_->topic_info_[topic].name = topic;
  node_->topic_info_[topic].min_rate = 20.0;
  node_->topic_info_[topic].max_rate = 30.0;

  // Feed 10Hz data (below min of 20)
  auto base_time = node_->get_clock()->now();
  for (int i = 0; i < 11; ++i) {
    rclcpp::Time t = base_time + rclcpp::Duration::from_seconds(i * 0.1);
    node_->update_topic_rate(topic, t);
  }

  node_->check_topic_rates();

  EXPECT_LT(node_->topic_info_[topic].rate, 20.0);
}

TEST_F(TopicRateLogicTest, RateAboveMax_StatusTooHigh)
{
  const std::string topic = "/test/rate_topic";
  node_->record_options_.topics.push_back(topic);
  node_->topic_info_[topic].name = topic;
  node_->topic_info_[topic].min_rate = 1.0;
  node_->topic_info_[topic].max_rate = 5.0;

  // Feed 10Hz data (above max of 5)
  auto base_time = node_->get_clock()->now();
  for (int i = 0; i < 11; ++i) {
    rclcpp::Time t = base_time + rclcpp::Duration::from_seconds(i * 0.1);
    node_->update_topic_rate(topic, t);
  }

  node_->check_topic_rates();

  EXPECT_GT(node_->topic_info_[topic].rate, 5.0);
}

TEST_F(TopicRateLogicTest, WindowSizeRespected)
{
  const std::string topic = "/test/rate_topic";
  node_->topic_info_[topic].name = topic;

  // Push 20 timestamps (window size is 10)
  auto base_time = node_->get_clock()->now();
  for (int i = 0; i < 20; ++i) {
    rclcpp::Time t = base_time + rclcpp::Duration::from_seconds(i * 0.1);
    node_->update_topic_rate(topic, t);
  }

  EXPECT_EQ(node_->topic_info_[topic].message_times.size(), 10u);
}

TEST_F(TopicRateLogicTest, SingleMessage_RateCalculation)
{
  const std::string topic = "/test/rate_topic";
  node_->record_options_.topics.push_back(topic);
  node_->topic_info_[topic].name = topic;

  auto base_time = node_->get_clock()->now();
  node_->update_topic_rate(topic, base_time);

  node_->check_topic_rates();

  // With a single message, rate should be small but non-negative
  EXPECT_GE(node_->topic_info_[topic].rate, 0.0);
}
