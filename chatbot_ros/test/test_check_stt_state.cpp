// Copyright (C) 2023 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
