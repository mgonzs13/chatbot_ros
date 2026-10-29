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

#ifndef CHATBOT_ROS__CHATBOT_CONFIG_HPP_
#define CHATBOT_ROS__CHATBOT_CONFIG_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace chatbot_ros {

/// Runtime configuration of the chatbot, loaded from ROS 2 parameters.
///
/// Every field has a sensible default so the chatbot can also be used
/// programmatically (e.g. from tests) without a parameter server.
struct ChatbotConfig {
  ChatbotConfig();

  /// Message spoken when the chatbot starts.
  std::string greeting{"Hi, how can I help you?"};

  /// System prompt sent with every chat completion request. An empty
  /// string disables the system message.
  std::string system_prompt;

  /// Messages spoken whenever the recognized speech is empty or noise,
  /// picked in round-robin order.
  std::vector<std::string> retry_messages;

  /// Initial prompt passed to Whisper with every STT request.
  std::string stt_prompt;

  /// Phrases (lower case) that are discarded when they show up as a
  /// transcription, e.g. common Whisper hallucinations.
  std::vector<std::string> stt_ignored_phrases;

  /// Default sampling temperature for the chat completions.
  double temperature{0.4};

  /// Maximum number of user/assistant turn pairs sent back to the LLM.
  int history_max_turns{8};

  /// Enable listening for user speech while the chatbot speaks
  /// (barge-in / interruption).
  bool interrupt_enabled{true};

  /// Similarity above which a transcription is considered an echo of the
  /// robot's own voice instead of a real interruption.
  double interrupt_similarity_threshold{0.8};

  /// Command words that always count as a real interruption when they
  /// appear in a short transcription (e.g. "stop").
  std::vector<std::string> interrupt_always_words;

  /// Enable the embeddings-based long-term memory.
  bool memory_enabled{false};

  /// Name of the llama_ros `GenerateEmbeddings` service.
  std::string memory_service{"/embedding/generate_embeddings"};

  /// JSON file used to persist the long-term memory. Empty selects
  /// `$HOME/.cache/chatbot_ros/memory.json`.
  std::string memory_storage_file;

  /// Number of memories injected into the system prompt.
  int memory_retrieve_k{3};

  /// Minimum cosine similarity for a memory to be retrieved.
  double memory_min_similarity{0.55};

  /// Maximum number of memories kept on disk.
  int memory_max_entries{512};

  /// Builds the configuration from the parameters of a node. Parameters
  /// that are not declared yet are declared with their default value.
  static ChatbotConfig from_node(const rclcpp::Node::SharedPtr &node);
};

/// Default voice-oriented system prompt. Kept short: it is sent on every
/// request and small models follow short prompts more reliably.
const char *default_system_prompt();

/// Default messages for a failed transcription attempt.
std::vector<std::string> default_retry_messages();

/// Default list of Whisper hallucination phrases to discard.
std::vector<std::string> default_stt_ignored_phrases();

} // namespace chatbot_ros

#endif // CHATBOT_ROS__CHATBOT_CONFIG_HPP_
