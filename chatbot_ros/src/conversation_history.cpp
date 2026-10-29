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
