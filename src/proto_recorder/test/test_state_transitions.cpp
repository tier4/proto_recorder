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

class StateTransitionsTest : public ::testing::Test
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

TEST_F(StateTransitionsTest, InitialState_NotRecording)
{
  EXPECT_FALSE(node_->is_recording_.load());
  EXPECT_FALSE(node_->is_paused());
}

TEST_F(StateTransitionsTest, PauseWithoutRecording_NoOp)
{
  node_->pause();
  EXPECT_FALSE(node_->is_paused());
}

TEST_F(StateTransitionsTest, ResumeWithoutRecording_NoOp)
{
  node_->resume();
  EXPECT_FALSE(node_->is_paused());
}

TEST_F(StateTransitionsTest, StartThenPause_IsPaused)
{
  // Use temp directory for bag output
  node_->original_uri_prefix_ = "/tmp/proto_recorder_test_state";
  node_->start();
  EXPECT_TRUE(node_->is_recording_.load());

  node_->pause();
  EXPECT_TRUE(node_->is_paused());

  // Cleanup
  node_->stop();
}

TEST_F(StateTransitionsTest, StartThenPauseThenResume_NotPaused)
{
  node_->original_uri_prefix_ = "/tmp/proto_recorder_test_resume";
  node_->start();
  node_->pause();
  EXPECT_TRUE(node_->is_paused());

  node_->resume();
  EXPECT_FALSE(node_->is_paused());

  node_->stop();
}

TEST_F(StateTransitionsTest, DoubleStart_NoError)
{
  node_->original_uri_prefix_ = "/tmp/proto_recorder_test_double_start";
  node_->start();
  EXPECT_NO_THROW(node_->start());  // Should warn but not throw

  node_->stop();
}

TEST_F(StateTransitionsTest, DoubleStop_NoError)
{
  EXPECT_NO_THROW(node_->stop());  // Should warn but not throw
}

TEST_F(StateTransitionsTest, DoublePause_NoError)
{
  node_->original_uri_prefix_ = "/tmp/proto_recorder_test_double_pause";
  node_->start();
  node_->pause();
  EXPECT_NO_THROW(node_->pause());  // Should warn but not throw

  node_->stop();
}

TEST_F(StateTransitionsTest, StopResetsState)
{
  node_->original_uri_prefix_ = "/tmp/proto_recorder_test_stop_reset";
  node_->start();
  node_->pause();
  node_->stop();

  EXPECT_FALSE(node_->is_recording_.load());
  EXPECT_FALSE(node_->is_paused());
}
