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
