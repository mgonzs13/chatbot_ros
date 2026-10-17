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

#ifndef CHATBOT_ROS__CONVERSATION_HISTORY_HPP_
#define CHATBOT_ROS__CONVERSATION_HISTORY_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "llama_msgs/msg/chat_message.hpp"

namespace chatbot_ros {

/// Short-term memory of the chatbot: the recent user and assistant
/// messages that are sent back to the LLM with every request.
class ConversationHistory {
public:
  using SharedPtr = std::shared_ptr<ConversationHistory>;

  void add_user_message(const std::string &content);
  void add_assistant_message(const std::string &content);

  /// Returns the most recent messages, keeping at most `max_turns`
  /// user/assistant pairs. The system prompt is not part of the history.
  std::vector<llama_msgs::msg::ChatMessage> messages(int max_turns) const;

  void clear();

private:
  void add_message(const std::string &role, const std::string &content);

  mutable std::mutex mutex_;
  std::deque<llama_msgs::msg::ChatMessage> messages_;
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__CONVERSATION_HISTORY_HPP_
