// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "chatbot_ros/states/check_stt_state.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

using chatbot_ros::CheckSttState;

TEST(CheckSttState, SucceedsWithNonEmptyTranscript) {
  CheckSttState state;
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "hello");
  EXPECT_EQ(state(blackboard), yasmin_ros::basic_outcomes::SUCCEED);
}

TEST(CheckSttState, AbortsAndSetsComplaintWithEmptyTranscript) {
  CheckSttState state({"first complaint", "second complaint"});
  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "");
  EXPECT_EQ(state(blackboard), yasmin_ros::basic_outcomes::ABORT);
  ASSERT_TRUE(blackboard->contains("tts"));
  EXPECT_EQ(blackboard->get<std::string>("tts"), "first complaint");
}

TEST(CheckSttState, RotatesComplaintMessages) {
  CheckSttState state({"first complaint", "second complaint"});
  auto blackboard = yasmin::Blackboard::make_shared();

  EXPECT_EQ(state(blackboard), yasmin_ros::basic_outcomes::ABORT);
  EXPECT_EQ(blackboard->get<std::string>("tts"), "first complaint");
  EXPECT_EQ(state(blackboard), yasmin_ros::basic_outcomes::ABORT);
  EXPECT_EQ(blackboard->get<std::string>("tts"), "second complaint");
  EXPECT_EQ(state(blackboard), yasmin_ros::basic_outcomes::ABORT);
  EXPECT_EQ(blackboard->get<std::string>("tts"), "first complaint");
}

TEST(CheckSttState, AbortsAndSetsComplaintWhenTranscriptMissing) {
  CheckSttState state({"please repeat"});
  auto blackboard = yasmin::Blackboard::make_shared();
  EXPECT_EQ(state(blackboard), yasmin_ros::basic_outcomes::ABORT);
  ASSERT_TRUE(blackboard->contains("tts"));
  EXPECT_EQ(blackboard->get<std::string>("tts"), "please repeat");
}

TEST(CheckSttState, UsesDefaultMessagesWhenNoneGiven) {
  CheckSttState state;
  auto blackboard = yasmin::Blackboard::make_shared();
  EXPECT_EQ(state(blackboard), yasmin_ros::basic_outcomes::ABORT);
  EXPECT_FALSE(blackboard->get<std::string>("tts").empty());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
