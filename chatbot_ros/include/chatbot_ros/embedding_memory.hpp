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
