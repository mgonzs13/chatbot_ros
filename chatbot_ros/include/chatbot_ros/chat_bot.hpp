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

/// Outcome of the responding concurrence when the user interrupted the
/// robot while it was speaking.
extern const char *const INTERRUPTED_OUTCOME;

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
