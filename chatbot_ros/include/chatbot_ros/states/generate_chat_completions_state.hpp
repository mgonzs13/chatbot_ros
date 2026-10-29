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

#ifndef CHATBOT_ROS__STATES__GENERATE_CHAT_COMPLETIONS_STATE_HPP_
#define CHATBOT_ROS__STATES__GENERATE_CHAT_COMPLETIONS_STATE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "chatbot_ros/chatbot_config.hpp"
#include "chatbot_ros/conversation_history.hpp"
#include "chatbot_ros/embedding_memory.hpp"
#include "chatbot_ros/states/sentence_accumulator.hpp"
#include "chatbot_ros/states/speech_pipeline.hpp"
#include "llama_msgs/action/generate_chat_completions.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_ros/action_state.hpp"

namespace chatbot_ros {

class GenerateChatCompletionsState
    : public yasmin_ros::ActionState<
          llama_msgs::action::GenerateChatCompletions> {
public:
  explicit GenerateChatCompletionsState(
      SpeechPipeline::SharedPtr pipeline,
      const ChatbotConfig &config = ChatbotConfig(),
      ConversationHistory::SharedPtr history = nullptr,
      EmbeddingMemory::SharedPtr memory = nullptr);

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

  /// Cancels the in-flight generation, used when the user interrupts.
  void request_cancel();

  /// Drops buffered text and rejects callbacks from the previous cycle.
  void begin_new_cycle();

protected:
  llama_msgs::action::GenerateChatCompletions::Goal
  create_goal_handler(yasmin::Blackboard::SharedPtr blackboard);

  void feedback_handler(
      yasmin::Blackboard::SharedPtr blackboard,
      std::shared_ptr<
          const llama_msgs::action::GenerateChatCompletions::Feedback>
          feedback);

private:
  std::string result_handler(
      yasmin::Blackboard::SharedPtr blackboard,
      llama_msgs::action::GenerateChatCompletions::Result::SharedPtr result);

  /// System prompt plus any retrieved long-term memories.
  std::string build_system_prompt(const std::string &user_text);

  SpeechPipeline::SharedPtr pipeline_;
  ChatbotConfig config_;
  ConversationHistory::SharedPtr history_;
  EmbeddingMemory::SharedPtr memory_;
  std::mutex buffer_mutex_;
  SentenceAccumulator accumulator_;
  std::string current_user_text_;
  /// Generation counter used to reject callbacks from a canceled cycle.
  uint64_t generation_{0};
  uint64_t goal_generation_{0};
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__GENERATE_CHAT_COMPLETIONS_STATE_HPP_
