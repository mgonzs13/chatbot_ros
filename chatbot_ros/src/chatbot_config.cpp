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

#include "chatbot_ros/chatbot_config.hpp"

#include <fstream>
#include <sstream>

namespace chatbot_ros {

namespace {

template <typename T>
T declare_or_get(const rclcpp::Node::SharedPtr &node, const std::string &name,
                 const T &default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

std::string read_file(const rclcpp::Node::SharedPtr &node,
                      const std::string &path) {
  std::ifstream file(path);
  if (!file) {
    RCLCPP_WARN(node->get_logger(), "Could not read system prompt file '%s'",
                path.c_str());
    return "";
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  std::string content = buffer.str();
  while (!content.empty() &&
         (content.back() == '\n' || content.back() == '\r')) {
    content.pop_back();
  }
  return content;
}

} // namespace

ChatbotConfig::ChatbotConfig() {
  this->system_prompt = default_system_prompt();
  this->retry_messages = default_retry_messages();
  this->stt_ignored_phrases = default_stt_ignored_phrases();
}

const char *default_system_prompt() {
  return "You are a friendly robot assistant that talks with people out "
         "loud. Follow these rules at all times:\n"
         "- Keep every answer short and natural, one or two sentences.\n"
         "- Use plain spoken language. Never use markdown, bullet points, "
         "emojis, code blocks, tables or any other formatting.\n"
         "- If you do not know something, say it briefly instead of making "
         "things up.\n"
         "- Reply in the same language the user is speaking.";
}

std::vector<std::string> default_retry_messages() {
  return {
      "Sorry, I didn't hear anything. Could you repeat that?",
      "I'm not sure I understood. Could you say it again?",
      "My ears must have missed that. Could you rephrase it?",
  };
}

std::vector<std::string> default_stt_ignored_phrases() {
  return {
      "thank you for watching",    "thanks for watching",
      "thank you for listening",   "subtitles by",
      "subtitles are provided by", "amara.org",
      "please subscribe",          "subscribe to my channel",
  };
}

ChatbotConfig ChatbotConfig::from_node(const rclcpp::Node::SharedPtr &node) {
  ChatbotConfig config;

  config.greeting =
      declare_or_get<std::string>(node, "messages.greeting", config.greeting);

  std::string system_prompt_file;
  config.system_prompt = declare_or_get<std::string>(
      node, "messages.system_prompt", default_system_prompt());
  system_prompt_file =
      declare_or_get<std::string>(node, "messages.system_prompt_file", "");
  if (config.system_prompt.empty() && !system_prompt_file.empty()) {
    config.system_prompt = read_file(node, system_prompt_file);
  }

  config.retry_messages = declare_or_get<std::vector<std::string>>(
      node, "messages.retry_messages", default_retry_messages());
  if (config.retry_messages.empty()) {
    config.retry_messages = default_retry_messages();
  }

  config.stt_prompt =
      declare_or_get<std::string>(node, "stt.prompt", config.stt_prompt);
  config.stt_ignored_phrases = declare_or_get<std::vector<std::string>>(
      node, "stt.ignored_phrases", default_stt_ignored_phrases());

  config.temperature =
      declare_or_get<double>(node, "llm.temperature", config.temperature);
  config.history_max_turns =
      declare_or_get<int>(node, "history.max_turns", config.history_max_turns);

  config.memory_enabled =
      declare_or_get<bool>(node, "memory.enabled", config.memory_enabled);
  config.memory_service = declare_or_get<std::string>(node, "memory.service",
                                                      config.memory_service);
  config.memory_storage_file = declare_or_get<std::string>(
      node, "memory.storage_file", config.memory_storage_file);
  config.memory_retrieve_k =
      declare_or_get<int>(node, "memory.retrieve_k", config.memory_retrieve_k);
  config.memory_min_similarity = declare_or_get<double>(
      node, "memory.min_similarity", config.memory_min_similarity);
  config.memory_max_entries = declare_or_get<int>(node, "memory.max_entries",
                                                  config.memory_max_entries);

  return config;
}

} // namespace chatbot_ros
