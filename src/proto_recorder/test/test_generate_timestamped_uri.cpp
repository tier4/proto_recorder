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
#include <regex>
#include <string>

class GenerateTimestampedUriTest : public ::testing::Test
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

TEST_F(GenerateTimestampedUriTest, ContainsPrefix)
{
  auto uri = node_->generate_timestamped_uri("my_bag");
  EXPECT_EQ(uri.substr(0, 7), "my_bag_");
}

TEST_F(GenerateTimestampedUriTest, ContainsDateComponents)
{
  auto uri = node_->generate_timestamped_uri("bag");

  // Expected format: bag_YYYY-MM-DDTHH-MM-SS+HHMM (or -HHMM)
  std::regex pattern(R"(bag_\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2}[+-]\d{4})");
  EXPECT_TRUE(std::regex_match(uri, pattern)) << "URI did not match expected format: " << uri;
}

TEST_F(GenerateTimestampedUriTest, DifferentPrefixes)
{
  auto uri_a = node_->generate_timestamped_uri("prefix_a");
  auto uri_b = node_->generate_timestamped_uri("prefix_b");

  EXPECT_NE(uri_a, uri_b);
  EXPECT_EQ(uri_a.substr(0, 9), "prefix_a_");
  EXPECT_EQ(uri_b.substr(0, 9), "prefix_b_");
}
