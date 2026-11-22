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

#include <string>
#include <vector>

#include "chatbot_ros/states/sentence_accumulator.hpp"

using chatbot_ros::SentenceAccumulator;

TEST(SentenceAccumulator, IncompleteTextProducesNothing) {
  SentenceAccumulator accumulator;
  EXPECT_TRUE(accumulator.add("Hello").empty());
  EXPECT_TRUE(accumulator.add(" world").empty());
}

TEST(SentenceAccumulator, PunctuationEmitsWholeBuffer) {
  SentenceAccumulator accumulator;
  accumulator.add("Hello");
  auto sentences = accumulator.add(" world.");
  ASSERT_EQ(sentences.size(), 1u);
  EXPECT_EQ(sentences[0], "Hello world.");
}

TEST(SentenceAccumulator, AllPunctuationMarksTrigger) {
  SentenceAccumulator accumulator;
  auto s1 = accumulator.add("Hi!");
  auto s2 = accumulator.add("Why?");
  auto s3 = accumulator.add("Note:");
  ASSERT_EQ(s1.size(), 1u);
  EXPECT_EQ(s1[0], "Hi!");
  ASSERT_EQ(s2.size(), 1u);
  EXPECT_EQ(s2[0], "Why?");
  ASSERT_EQ(s3.size(), 1u);
  EXPECT_EQ(s3[0], "Note:");
}

TEST(SentenceAccumulator, FlushReturnsLeftover) {
  SentenceAccumulator accumulator;
  accumulator.add("Bye");
  auto leftover = accumulator.flush();
  ASSERT_TRUE(leftover.has_value());
  EXPECT_EQ(*leftover, "Bye");
  EXPECT_FALSE(accumulator.flush().has_value());
}

TEST(SentenceAccumulator, FlushIgnoresWhitespaceOnly) {
  SentenceAccumulator accumulator;
  accumulator.add("   ");
  EXPECT_FALSE(accumulator.flush().has_value());
}

TEST(SentenceAccumulator, PunctuationAfterWhitespaceEmitsRawBuffer) {
  SentenceAccumulator accumulator;
  auto sentences = accumulator.add("  Hello there!  ");
  ASSERT_EQ(sentences.size(), 1u);
  EXPECT_EQ(sentences[0], "  Hello there!  ");
}
