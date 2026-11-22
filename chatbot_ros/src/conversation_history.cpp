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

#include "chatbot_ros/conversation_history.hpp"

namespace chatbot_ros {

void ConversationHistory::add_user_message(const std::string &content) {
  this->add_message("user", content);
}

void ConversationHistory::add_assistant_message(const std::string &content) {
  this->add_message("assistant", content);
}

void ConversationHistory::add_message(const std::string &role,
                                      const std::string &content) {
  if (content.empty()) {
    return;
  }
  llama_msgs::msg::ChatMessage message;
  message.role = role;
  message.content = content;

  std::lock_guard<std::mutex> lock(this->mutex_);
  this->messages_.push_back(message);
}

std::vector<llama_msgs::msg::ChatMessage>
ConversationHistory::messages(int max_turns) const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  std::vector<llama_msgs::msg::ChatMessage> result;
  if (max_turns <= 0) {
    return result;
  }

  // Each turn usually holds one user and one assistant message, but
  // interrupted turns only hold the user message, so trim by count.
  const std::size_t limit = static_cast<std::size_t>(max_turns) * 2;
  const std::size_t begin =
      (this->messages_.size() > limit) ? this->messages_.size() - limit : 0;
  result.reserve(this->messages_.size() - begin);
  for (std::size_t i = begin; i < this->messages_.size(); i++) {
    result.push_back(this->messages_[i]);
  }
  return result;
}

void ConversationHistory::clear() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->messages_.clear();
}

} // namespace chatbot_ros
