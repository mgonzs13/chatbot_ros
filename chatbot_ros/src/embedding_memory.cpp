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

#include "chatbot_ros/embedding_memory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <utility>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace chatbot_ros {

namespace {
constexpr int kEmbeddingTimeoutMs = 3000;
} // namespace

EmbeddingMemory::EmbeddingMemory(const ChatbotConfig &config,
                                 const rclcpp::Node::SharedPtr &node)
    : config_(config), node_(node),
      storage_file_(config.memory_storage_file.empty()
                        ? default_storage_file()
                        : config.memory_storage_file) {
  if (this->config_.memory_enabled && this->node_ != nullptr) {
    this->client_ =
        this->node_->create_client<llama_msgs::srv::GenerateEmbeddings>(
            this->config_.memory_service);
  }
}

std::string EmbeddingMemory::default_storage_file() {
  const char *home = std::getenv("HOME");
  const std::string base = (home != nullptr) ? home : ".";
  return base + "/.cache/chatbot_ros/memory.json";
}

double EmbeddingMemory::cosine_similarity(const std::vector<float> &a,
                                          const std::vector<float> &b) {
  if (a.size() != b.size() || a.empty()) {
    return 0.0;
  }
  double dot = 0.0;
  double norm_a = 0.0;
  double norm_b = 0.0;
  for (std::size_t i = 0; i < a.size(); i++) {
    dot += static_cast<double>(a[i]) * static_cast<double>(b[i]);
    norm_a += static_cast<double>(a[i]) * static_cast<double>(a[i]);
    norm_b += static_cast<double>(b[i]) * static_cast<double>(b[i]);
  }
  if (norm_a <= 0.0 || norm_b <= 0.0) {
    return 0.0;
  }
  return dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
}

void EmbeddingMemory::set_embedder(Embedder embedder) {
  std::lock_guard<std::mutex> lock(this->embed_mutex_);
  this->embedder_ = std::move(embedder);
}

std::optional<std::vector<float>>
EmbeddingMemory::embed(const std::string &text) {
  std::lock_guard<std::mutex> lock(this->embed_mutex_);
  if (this->embedder_) {
    return this->embedder_(text);
  }
  if (this->client_ == nullptr || text.empty()) {
    return std::nullopt;
  }
  return this->embed_via_service(text);
}

std::optional<std::vector<float>>
EmbeddingMemory::embed_via_service(const std::string &text) {
  if (!this->client_->service_is_ready()) {
    if (!this->client_->wait_for_service(std::chrono::milliseconds(500))) {
      if (!this->warned_service_) {
        RCLCPP_WARN(this->node_->get_logger(),
                    "Embeddings service '%s' is not available; long-term "
                    "memory is disabled",
                    this->config_.memory_service.c_str());
        this->warned_service_ = true;
      }
      return std::nullopt;
    }
    this->warned_service_ = false;
  }

  auto request =
      std::make_shared<llama_msgs::srv::GenerateEmbeddings::Request>();
  request->prompt = text;

  try {
    auto future = this->client_->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(kEmbeddingTimeoutMs)) !=
        std::future_status::ready) {
      RCLCPP_WARN(this->node_->get_logger(),
                  "Embeddings service timed out after %d ms",
                  kEmbeddingTimeoutMs);
      return std::nullopt;
    }
    auto response = future.get();
    if (response == nullptr || response->embeddings.empty()) {
      return std::nullopt;
    }
    return response->embeddings;
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->node_->get_logger(), "Embeddings request failed: %s",
                e.what());
    return std::nullopt;
  }
}

void EmbeddingMemory::ensure_loaded_locked() {
  if (this->loaded_) {
    return;
  }
  this->loaded_ = true;
  this->load_locked();
}

void EmbeddingMemory::load_locked() {
  std::error_code error;
  if (!std::filesystem::exists(this->storage_file_, error)) {
    return;
  }

  try {
    std::ifstream file(this->storage_file_);
    if (!file) {
      return;
    }
    json data;
    file >> data;

    this->entries_.clear();
    for (const auto &item : data.value("entries", json::array())) {
      Entry entry;
      entry.text = item.value("text", "");
      entry.timestamp = item.value("timestamp", static_cast<int64_t>(0));
      entry.embedding = item.value("embedding", std::vector<float>());
      if (!entry.text.empty() && !entry.embedding.empty()) {
        this->entries_.push_back(std::move(entry));
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->node_ ? this->node_->get_logger()
                            : rclcpp::get_logger("chatbot_ros"),
                "Could not load long-term memory from '%s': %s",
                this->storage_file_.c_str(), e.what());
    this->entries_.clear();
  }
}

void EmbeddingMemory::save_locked() {
  const std::string parent =
      std::filesystem::path(this->storage_file_).parent_path().string();
  if (!parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
  }

  json data;
  data["version"] = 1;
  data["service"] = this->config_.memory_service;
  json entries = json::array();
  for (const auto &entry : this->entries_) {
    json item;
    item["text"] = entry.text;
    item["timestamp"] = entry.timestamp;
    item["embedding"] = entry.embedding;
    entries.push_back(std::move(item));
  }
  data["entries"] = std::move(entries);

  const std::string temporary = this->storage_file_ + ".tmp";
  try {
    std::ofstream file(temporary);
    if (!file) {
      RCLCPP_WARN(this->node_ ? this->node_->get_logger()
                              : rclcpp::get_logger("chatbot_ros"),
                  "Could not write long-term memory to '%s'",
                  temporary.c_str());
      return;
    }
    file << data.dump();
    file.close();
    std::filesystem::rename(temporary, this->storage_file_);
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->node_ ? this->node_->get_logger()
                            : rclcpp::get_logger("chatbot_ros"),
                "Could not persist long-term memory: %s", e.what());
  }
}

void EmbeddingMemory::remember_turn(const std::string &user_text,
                                    const std::string &assistant_text) {
  if (!this->enabled() || user_text.empty() || assistant_text.empty()) {
    return;
  }

  const std::string text = "The user asked: " + user_text +
                           "\nThe assistant answered: " + assistant_text;
  std::optional<std::vector<float>> embedding = this->embed(text);
  if (!embedding.has_value()) {
    return;
  }

  const int64_t timestamp =
      std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  std::lock_guard<std::mutex> lock(this->mutex_);
  this->ensure_loaded_locked();
  this->entries_.push_back(Entry{text, std::move(*embedding), timestamp});

  if (this->config_.memory_max_entries > 0 &&
      static_cast<int>(this->entries_.size()) >
          this->config_.memory_max_entries) {
    const std::size_t excess =
        this->entries_.size() - this->config_.memory_max_entries;
    this->entries_.erase(this->entries_.begin(),
                         this->entries_.begin() + excess);
  }
  this->save_locked();
}

std::vector<std::string> EmbeddingMemory::retrieve(const std::string &query) {
  if (!this->enabled() || query.empty() ||
      this->config_.memory_retrieve_k <= 0) {
    return {};
  }

  std::optional<std::vector<float>> query_embedding = this->embed(query);
  if (!query_embedding.has_value()) {
    return {};
  }

  std::lock_guard<std::mutex> lock(this->mutex_);
  this->ensure_loaded_locked();

  std::vector<std::pair<double, std::size_t>> scored;
  for (std::size_t i = 0; i < this->entries_.size(); i++) {
    const double score =
        cosine_similarity(*query_embedding, this->entries_[i].embedding);
    if (score >= this->config_.memory_min_similarity) {
      scored.emplace_back(score, i);
    }
  }
  std::sort(scored.begin(), scored.end(),
            [](const auto &a, const auto &b) { return a.first > b.first; });

  std::vector<std::string> result;
  for (std::size_t i = 0; i < scored.size() &&
                          static_cast<int>(i) < this->config_.memory_retrieve_k;
       i++) {
    result.push_back(this->entries_[scored[i].second].text);
  }
  return result;
}

} // namespace chatbot_ros
