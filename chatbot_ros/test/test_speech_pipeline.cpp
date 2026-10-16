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

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include "chatbot_ros/states/speech_pipeline.hpp"

using namespace std::chrono_literals;
using chatbot_ros::SpeechPipeline;

TEST(SpeechPipeline, PopReturnsPushedSentence) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->push("Hello.");
  std::string sentence;
  ASSERT_TRUE(pipeline->pop(sentence));
  EXPECT_EQ(sentence, "Hello.");
}

TEST(SpeechPipeline, PopBlocksUntilPush) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  auto future = std::async(std::launch::async, [pipeline]() {
    std::string sentence;
    bool ok = pipeline->pop(sentence);
    return std::make_pair(ok, sentence);
  });
  EXPECT_EQ(future.wait_for(100ms), std::future_status::timeout);
  pipeline->push("Hi.");
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  auto [ok, sentence] = future.get();
  EXPECT_TRUE(ok);
  EXPECT_EQ(sentence, "Hi.");
}

TEST(SpeechPipeline, ClosedEmptyStopsPop) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->close("succeeded");
  std::string sentence;
  EXPECT_FALSE(pipeline->pop(sentence));
}

TEST(SpeechPipeline, DrainsRemainingSentencesBeforeStopping) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->push("One.");
  pipeline->close("succeeded");
  std::string sentence;
  ASSERT_TRUE(pipeline->pop(sentence));
  EXPECT_EQ(sentence, "One.");
  EXPECT_FALSE(pipeline->pop(sentence));
}

TEST(SpeechPipeline, CloseKeepsFirstOutcome) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->close("succeeded");
  pipeline->close("aborted");
  EXPECT_TRUE(pipeline->is_closed());
  EXPECT_EQ(pipeline->get_generation_outcome(), "succeeded");
}

TEST(SpeechPipeline, CancelStopsPopEvenWithQueuedSentences) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->push("One.");
  pipeline->cancel();
  std::string sentence;
  EXPECT_TRUE(pipeline->is_canceled());
  EXPECT_FALSE(pipeline->pop(sentence));
}

TEST(SpeechPipeline, CloseWakesBlockedPop) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  auto future = std::async(std::launch::async, [pipeline]() {
    std::string sentence;
    return pipeline->pop(sentence);
  });
  EXPECT_EQ(future.wait_for(100ms), std::future_status::timeout);
  pipeline->close("succeeded");
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(future.get());
}

TEST(SpeechPipeline, CancelWakesBlockedPop) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  auto future = std::async(std::launch::async, [pipeline]() {
    std::string sentence;
    return pipeline->pop(sentence);
  });
  EXPECT_EQ(future.wait_for(100ms), std::future_status::timeout);
  pipeline->cancel();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(future.get());
}

TEST(SpeechPipeline, PushAfterCloseIsDropped) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->close("succeeded");
  pipeline->push("Too late.");
  std::string sentence;
  EXPECT_FALSE(pipeline->pop(sentence));
}

TEST(SpeechPipeline, FifoOrderIsPreserved) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->push("First.");
  pipeline->push("Second.");
  std::string sentence;
  ASSERT_TRUE(pipeline->pop(sentence));
  EXPECT_EQ(sentence, "First.");
  ASSERT_TRUE(pipeline->pop(sentence));
  EXPECT_EQ(sentence, "Second.");
}

TEST(SpeechPipeline, ResetAllowsReuse) {
  auto pipeline = std::make_shared<SpeechPipeline>();
  pipeline->push("Old.");
  pipeline->close("succeeded");
  pipeline->reset();
  EXPECT_FALSE(pipeline->is_closed());
  EXPECT_FALSE(pipeline->is_canceled());
  EXPECT_EQ(pipeline->get_generation_outcome(), "");
  pipeline->push("New.");
  std::string sentence;
  ASSERT_TRUE(pipeline->pop(sentence));
  EXPECT_EQ(sentence, "New.");
  EXPECT_FALSE(pipeline->is_closed());
}
