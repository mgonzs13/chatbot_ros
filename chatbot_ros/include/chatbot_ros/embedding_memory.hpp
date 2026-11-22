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

#ifndef CHATBOT_ROS__EMBEDDING_MEMORY_HPP_
#define CHATBOT_ROS__EMBEDDING_MEMORY_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "chatbot_ros/chatbot_config.hpp"
#include "llama_msgs/srv/generate_embeddings.hpp"

namespace chatbot_ros {

/// Long-term memory based on text embeddings.
///
/// Past conversation turns are embedded with a `llama_ros`
/// `GenerateEmbeddings` service and stored in a small JSON vector store.
/// The most similar entries are retrieved and injected into the system
/// prompt of new requests.
class EmbeddingMemory {
public:
  using SharedPtr = std::shared_ptr<EmbeddingMemory>;

  /// Backend used to embed texts. Defaults to the ROS service.
  using Embedder =
      std::function<std::optional<std::vector<float>>(const std::string &)>;

  EmbeddingMemory(const ChatbotConfig &config,
                  const rclcpp::Node::SharedPtr &node);

  bool enabled() const { return this->config_.memory_enabled; }

  /// Stores a finished conversation turn (best effort).
  void remember_turn(const std::string &user_text,
                     const std::string &assistant_text);

  /// Returns the texts of the most similar memories to `query`, most
  /// similar first.
  std::vector<std::string> retrieve(const std::string &query);

  /// Replaces the embeddings backend, mainly for testing.
  void set_embedder(Embedder embedder);

  static double cosine_similarity(const std::vector<float> &a,
                                  const std::vector<float> &b);

private:
  struct Entry {
    std::string text;
    std::vector<float> embedding;
    int64_t timestamp{0};
  };

  static std::string default_storage_file();

  std::optional<std::vector<float>> embed(const std::string &text);
  std::optional<std::vector<float>> embed_via_service(const std::string &text);

  void ensure_loaded_locked();
  void load_locked();
  void save_locked();

  ChatbotConfig config_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<llama_msgs::srv::GenerateEmbeddings>::SharedPtr client_;
  Embedder embedder_;
  std::string storage_file_;

  std::mutex mutex_;
  std::mutex embed_mutex_;
  std::vector<Entry> entries_;
  bool loaded_{false};
  bool warned_service_{false};
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__EMBEDDING_MEMORY_HPP_
