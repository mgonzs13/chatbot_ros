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

#include "chatbot_ros/conversation_history.hpp"
#include "chatbot_ros/echo_guard.hpp"
#include "chatbot_ros/text/speech_sanitizer.hpp"
#include "chatbot_ros/text/text_similarity.hpp"
#include "chatbot_ros/text/transcript_filter.hpp"

using chatbot_ros::ConversationHistory;
using chatbot_ros::EchoGuard;
using chatbot_ros::text::SpeechSanitizer;
using chatbot_ros::text::TranscriptFilter;

TEST(SpeechSanitizer, RemovesEmphasisAndInlineCode) {
  EXPECT_EQ(SpeechSanitizer::sanitize("Hello **world**!"), "Hello world!");
  EXPECT_EQ(SpeechSanitizer::sanitize("Use `code` here."), "Use code here.");
  EXPECT_EQ(SpeechSanitizer::sanitize("snake_case _name_"), "snake case name");
}

TEST(SpeechSanitizer, RemovesHeadingsAndLists) {
  EXPECT_EQ(SpeechSanitizer::sanitize("# Title\nSome text."),
            "Title Some text.");
  EXPECT_EQ(SpeechSanitizer::sanitize("Steps:\n- first\n- second"),
            "Steps: first second");
  EXPECT_EQ(SpeechSanitizer::sanitize("1. one\n2. two"), "one two");
  EXPECT_EQ(SpeechSanitizer::sanitize("> quoted line"), "quoted line");
}

TEST(SpeechSanitizer, KeepsLinkLabelsAndDropsUrls) {
  EXPECT_EQ(SpeechSanitizer::sanitize("Check [the docs](https://x.com) now."),
            "Check the docs now.");
  EXPECT_EQ(SpeechSanitizer::sanitize("Visit https://example.com for info."),
            "Visit for info.");
  EXPECT_EQ(SpeechSanitizer::sanitize("![image](https://x.com/p.png) done"),
            "done");
}

TEST(SpeechSanitizer, RemovesEmojiAndSymbols) {
  EXPECT_EQ(SpeechSanitizer::sanitize("Let's go 🚀 now!"), "Let's go now!");
  EXPECT_EQ(SpeechSanitizer::sanitize("Great ♪ work"), "Great work");
  EXPECT_EQ(SpeechSanitizer::sanitize("I ❤ robots"), "I robots");
}

TEST(SpeechSanitizer, ConvertsPunctuationAndSpacing) {
  EXPECT_EQ(SpeechSanitizer::sanitize("Wait — really?"), "Wait, really?");
  EXPECT_EQ(SpeechSanitizer::sanitize("Hello   world  ."), "Hello world.");
}

TEST(SpeechSanitizer, DropsCodeBlocks) {
  EXPECT_EQ(SpeechSanitizer::sanitize("```python\nprint('hi')\n```\nDone."),
            "Done.");
}

TEST(SpeechSanitizer, ReturnsEmptyForNonSpeakableText) {
  EXPECT_EQ(SpeechSanitizer::sanitize("***"), "");
  EXPECT_EQ(SpeechSanitizer::sanitize("..."), "");
  EXPECT_EQ(SpeechSanitizer::sanitize(""), "");
}

TEST(TranscriptFilter, RemovesNonSpeechAnnotations) {
  TranscriptFilter filter;
  EXPECT_EQ(filter.clean("[BLANK_AUDIO]"), "");
  EXPECT_EQ(filter.clean("(music)"), "");
  EXPECT_EQ(filter.clean("[SOUND] turn on the light"), "turn on the light");
  EXPECT_EQ(filter.clean("  Hello   world  "), "Hello world");
  EXPECT_EQ(filter.clean("*sighs* okay"), "okay");
  EXPECT_EQ(filter.clean_or_empty("[BLANK_AUDIO]"), "");
  EXPECT_EQ(filter.clean_or_empty("(coughs)"), "");
  EXPECT_EQ(filter.clean_or_empty("turn on the light"), "turn on the light");
}

TEST(TranscriptFilter, DropsIgnoredHallucinations) {
  TranscriptFilter filter({"thank you for watching", "subtitles by"});
  EXPECT_EQ(filter.clean_or_empty("Thank you for watching!"), "");
  EXPECT_EQ(filter.clean_or_empty("thank you"), "");
  EXPECT_EQ(filter.clean_or_empty("Subtitles by someone"), "");
  EXPECT_EQ(filter.clean_or_empty("Please open the door."),
            "Please open the door.");
  // clean() alone only strips annotations, so the hallucinated phrase
  // survives it and is only dropped by clean_or_empty().
  EXPECT_EQ(filter.clean("Thank you for watching!"), "Thank you for watching!");
}

TEST(TranscriptFilter, CleanOrEmptyCombinesCleaningAndNoiseFiltering) {
  TranscriptFilter filter({"thank you for watching"});
  EXPECT_EQ(filter.clean_or_empty("[BLANK_AUDIO]"), "");
  EXPECT_EQ(filter.clean_or_empty("(coughs)"), "");
  EXPECT_EQ(filter.clean_or_empty("Thank you for watching!"), "");
  EXPECT_EQ(filter.clean_or_empty("  Hello   robot  "), "Hello robot");
}

TEST(TranscriptFilter, RejectsTooShortTranscripts) {
  TranscriptFilter filter;
  EXPECT_EQ(filter.clean_or_empty("a"), "");
  EXPECT_EQ(filter.clean_or_empty("..."), "");
  EXPECT_EQ(filter.clean_or_empty("hi"), "hi");
}

TEST(TextSimilarity, ScoresWordOverlap) {
  EXPECT_DOUBLE_EQ(chatbot_ros::text::similarity("hello robot", "hello robot"),
                   1.0);
  EXPECT_DOUBLE_EQ(
      chatbot_ros::text::similarity("can help", "I can help you plan"), 1.0);
  EXPECT_DOUBLE_EQ(
      chatbot_ros::text::similarity("stop", "I can help you plan your day"),
      0.0);
  EXPECT_DOUBLE_EQ(chatbot_ros::text::similarity("hello world", "hello there"),
                   0.5);
}

TEST(TextSimilarity, SubsequenceToleratesMissingAndExtraWords) {
  // Extra hallucinated words at the end.
  EXPECT_NEAR(chatbot_ros::text::subsequence_similarity(
                  "i can do that thank you", "Sure, I can do that."),
              4.0 / 6.0, 1e-9);
  // Missing words.
  EXPECT_DOUBLE_EQ(
      chatbot_ros::text::subsequence_similarity(
          "once upon a time robot", "once upon a time there was a robot"),
      1.0);
  // Word order matters.
  EXPECT_NEAR(chatbot_ros::text::subsequence_similarity(
                  "can you help me", "Sure, I can help you with that."),
              0.5, 1e-9);
  EXPECT_DOUBLE_EQ(chatbot_ros::text::subsequence_similarity(
                       "turn on the light", "hello, how are you"),
                   0.0);
}

TEST(EchoGuard, DetectsEchoOfSpokenSentences) {
  EchoGuard guard;
  guard.add_spoken_sentence("I can help you plan your day.");
  EXPECT_TRUE(guard.is_echo("I can help you plan your day", 0.7));
  EXPECT_TRUE(guard.is_echo("can help you plan", 0.7));
  EXPECT_FALSE(guard.is_echo("turn on the light", 0.7));
}

TEST(EchoGuard, IgnoresEchoWithExtraHallucinatedWords) {
  EchoGuard guard;
  guard.add_spoken_sentence("Sure, I can do that.");
  EXPECT_TRUE(guard.is_echo("i can do that thank you", 0.65));
  EXPECT_FALSE(guard.is_echo("i can do that thank you", 0.8));
}

TEST(EchoGuard, DoesNotMatchDifferentRequests) {
  EchoGuard guard;
  guard.add_spoken_sentence("Sure, I can help you with that.");
  EXPECT_FALSE(guard.is_echo("can you help me", 0.65));
}

TEST(EchoGuard, SingleWordFragmentsOfSpokenTextAreEcho) {
  EchoGuard guard;
  guard.add_spoken_sentence("Sure, I can do that.");
  EXPECT_TRUE(guard.is_echo("Sure.", 0.65));
}

TEST(EchoGuard, ShortSingleWordsAreNeverEcho) {
  EchoGuard guard;
  guard.add_spoken_sentence("No, that is wrong.");
  EXPECT_FALSE(guard.is_echo("no", 0.65));
}

TEST(EchoGuard, AlwaysWordsInterruptEvenIfRecentlySpoken) {
  EchoGuard guard;
  guard.add_spoken_sentence("Please stop the music now.");
  EXPECT_FALSE(guard.is_echo("stop the music", 0.65, {"stop"}));
  EXPECT_TRUE(guard.is_echo("stop the music", 0.65));
  // Long transcriptions are not exempted by the command words, otherwise a
  // full echo of a sentence containing "stop" would never be filtered.
  EXPECT_TRUE(guard.is_echo("please stop the music now", 0.65, {"stop"}));
}

TEST(EchoGuard, ClearForgetsSentences) {
  EchoGuard guard;
  guard.add_spoken_sentence("Hello there robot.");
  guard.clear();
  EXPECT_FALSE(guard.is_echo("Hello there robot", 0.7));
}

TEST(ConversationHistory, BuildsUserAndAssistantMessages) {
  ConversationHistory history;
  history.add_user_message("hello");
  history.add_assistant_message("hi there");

  const auto messages = history.messages(8);
  ASSERT_EQ(messages.size(), 2u);
  EXPECT_EQ(messages[0].role, "user");
  EXPECT_EQ(messages[0].content, "hello");
  EXPECT_EQ(messages[1].role, "assistant");
  EXPECT_EQ(messages[1].content, "hi there");
}

TEST(ConversationHistory, TrimsToMaxTurns) {
  ConversationHistory history;
  for (int i = 0; i < 3; i++) {
    history.add_user_message("user " + std::to_string(i));
    history.add_assistant_message("assistant " + std::to_string(i));
  }

  const auto messages = history.messages(1);
  ASSERT_EQ(messages.size(), 2u);
  EXPECT_EQ(messages[0].content, "user 2");
  EXPECT_EQ(messages[1].content, "assistant 2");
}

TEST(ConversationHistory, IgnoresEmptyMessages) {
  ConversationHistory history;
  history.add_user_message("");
  history.add_assistant_message("");
  EXPECT_TRUE(history.messages(8).empty());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
