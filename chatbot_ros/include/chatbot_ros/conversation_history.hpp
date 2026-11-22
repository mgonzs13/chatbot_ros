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
