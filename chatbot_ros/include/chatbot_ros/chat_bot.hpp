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

#ifndef CHATBOT_ROS__CHAT_BOT_HPP_
#define CHATBOT_ROS__CHAT_BOT_HPP_

#include <memory>
#include <string>

#include "chatbot_ros/chatbot_config.hpp"
#include "chatbot_ros/conversation_history.hpp"
#include "chatbot_ros/embedding_memory.hpp"
#include "chatbot_ros/states/speech_pipeline.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/state_machine.hpp"

namespace chatbot_ros {

yasmin::Concurrence::SharedPtr create_responding_concurrence(
    const SpeechPipeline::SharedPtr &pipeline,
    const ChatbotConfig &config = ChatbotConfig(),
    const ConversationHistory::SharedPtr &history = nullptr,
    const EmbeddingMemory::SharedPtr &memory = nullptr);

class ChatBot {
public:
  ChatBot();

  explicit ChatBot(const ChatbotConfig &config);

  std::string execute();

  yasmin::StateMachine::SharedPtr get_state_machine() const;

  const ChatbotConfig &get_config() const;

private:
  void build_state_machine();

  yasmin::StateMachine::SharedPtr state_machine_;
  SpeechPipeline::SharedPtr pipeline_;
  ChatbotConfig config_;
  ConversationHistory::SharedPtr history_;
  EmbeddingMemory::SharedPtr memory_;
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__CHAT_BOT_HPP_
