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

#include <memory>
#include <string>
#include <vector>

class LoadTopicsFromFileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.append_parameter_override("start_recording", false);
    node_ = std::make_shared<proto_recorder::ProtoRecorder>(options);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<proto_recorder::ProtoRecorder> node_;
};

TEST_F(LoadTopicsFromFileTest, LoadValidFile)
{
  std::string path = std::string(TEST_FIXTURES_DIR) + "/test_topics.yaml";
  auto topics = node_->load_topics_from_file(path);

  ASSERT_EQ(topics.size(), 3u);
  EXPECT_EQ(topics[0], "/test/topic_a");
  EXPECT_EQ(topics[1], "/test/topic_b");
  EXPECT_EQ(topics[2], "/test/topic_no_rate");

  // Check that rate info was loaded for topics with rate config
  EXPECT_DOUBLE_EQ(node_->topic_info_["/test/topic_a"].min_rate, 9.0);
  EXPECT_DOUBLE_EQ(node_->topic_info_["/test/topic_a"].max_rate, 11.0);
  EXPECT_DOUBLE_EQ(node_->topic_info_["/test/topic_b"].min_rate, 4.0);
  EXPECT_DOUBLE_EQ(node_->topic_info_["/test/topic_b"].max_rate, 6.0);

  // Topic without rate config should have default 0.0
  EXPECT_DOUBLE_EQ(node_->topic_info_["/test/topic_no_rate"].min_rate, 0.0);
  EXPECT_DOUBLE_EQ(node_->topic_info_["/test/topic_no_rate"].max_rate, 0.0);
}

TEST_F(LoadTopicsFromFileTest, LoadInvalidFile)
{
  std::string path = std::string(TEST_FIXTURES_DIR) + "/invalid_topics.yaml";
  auto topics = node_->load_topics_from_file(path);

  EXPECT_TRUE(topics.empty());
}

TEST_F(LoadTopicsFromFileTest, LoadNonexistentFile)
{
  auto topics = node_->load_topics_from_file("/nonexistent/path/topics.yaml");

  EXPECT_TRUE(topics.empty());
}
