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

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "chatbot_ros/chatbot_config.hpp"

using chatbot_ros::ChatbotConfig;

TEST(ChatbotConfig, DefaultsMatchParameterDefaults) {
  auto node = std::make_shared<rclcpp::Node>("chatbot_config_default_test");
  const ChatbotConfig config = ChatbotConfig::from_node(node);

  EXPECT_EQ(config.greeting, "Hi, how can I help you?");
  EXPECT_FALSE(config.system_prompt.empty());
  EXPECT_FALSE(config.retry_messages.empty());
  EXPECT_FALSE(config.stt_ignored_phrases.empty());
  EXPECT_TRUE(config.interrupt_enabled);
  EXPECT_FALSE(config.memory_enabled);
  EXPECT_EQ(config.history_max_turns, 8);
  EXPECT_DOUBLE_EQ(config.interrupt_similarity_threshold, 0.8);
  EXPECT_FALSE(config.interrupt_always_words.empty());
}

TEST(ChatbotConfig, ParameterOverridesAreApplied) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("messages.greeting", "Hello robot!"),
       rclcpp::Parameter("messages.system_prompt", "Be brief."),
       rclcpp::Parameter("messages.retry_messages",
                         std::vector<std::string>{"Say that again."}),
       rclcpp::Parameter("stt.prompt", "voice commands"),
       rclcpp::Parameter("llm.temperature", 0.7),
       rclcpp::Parameter("history.max_turns", 3),
       rclcpp::Parameter("interrupt.enabled", false),
       rclcpp::Parameter("interrupt.similarity_threshold", 0.5),
       rclcpp::Parameter("interrupt.always_words",
                         std::vector<std::string>{"halt", "freeze"}),
       rclcpp::Parameter("memory.enabled", true),
       rclcpp::Parameter("memory.service", "/embeddings/generate"),
       rclcpp::Parameter("memory.retrieve_k", 5),
       rclcpp::Parameter("memory.min_similarity", 0.25),
       rclcpp::Parameter("memory.max_entries", 42)});

  auto node =
      std::make_shared<rclcpp::Node>("chatbot_config_override_test", options);
  const ChatbotConfig config = ChatbotConfig::from_node(node);

  EXPECT_EQ(config.greeting, "Hello robot!");
  EXPECT_EQ(config.system_prompt, "Be brief.");
  ASSERT_EQ(config.retry_messages.size(), 1u);
  EXPECT_EQ(config.retry_messages[0], "Say that again.");
  EXPECT_EQ(config.stt_prompt, "voice commands");
  EXPECT_DOUBLE_EQ(config.temperature, 0.7);
  EXPECT_EQ(config.history_max_turns, 3);
  EXPECT_FALSE(config.interrupt_enabled);
  EXPECT_DOUBLE_EQ(config.interrupt_similarity_threshold, 0.5);
  ASSERT_EQ(config.interrupt_always_words.size(), 2u);
  EXPECT_EQ(config.interrupt_always_words[0], "halt");
  EXPECT_EQ(config.interrupt_always_words[1], "freeze");
  EXPECT_TRUE(config.memory_enabled);
  EXPECT_EQ(config.memory_service, "/embeddings/generate");
  EXPECT_EQ(config.memory_retrieve_k, 5);
  EXPECT_DOUBLE_EQ(config.memory_min_similarity, 0.25);
  EXPECT_EQ(config.memory_max_entries, 42);
}

TEST(ChatbotConfig, LoadsSystemPromptFromFile) {
  const std::string path =
      "/tmp/chatbot_prompt_" + std::to_string(getpid()) + ".txt";
  {
    std::ofstream file(path);
    file << "You are a test robot.\n";
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("messages.system_prompt", ""),
       rclcpp::Parameter("messages.system_prompt_file", path)});

  auto node =
      std::make_shared<rclcpp::Node>("chatbot_config_file_test", options);
  const ChatbotConfig config = ChatbotConfig::from_node(node);

  EXPECT_EQ(config.system_prompt, "You are a test robot.");
  std::remove(path.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
