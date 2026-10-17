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

#include "chatbot_ros/states/generate_chat_completions_state.hpp"

#include <functional>
#include <utility>
#include <vector>

#include "chatbot_ros/text/speech_sanitizer.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace chatbot_ros {

GenerateChatCompletionsState::GenerateChatCompletionsState(
    SpeechPipeline::SharedPtr pipeline, const ChatbotConfig &config,
    ConversationHistory::SharedPtr history, EmbeddingMemory::SharedPtr memory)
    : yasmin_ros::ActionState<llama_msgs::action::GenerateChatCompletions>(
          "/llama/generate_chat_completions",
          std::bind(&GenerateChatCompletionsState::create_goal_handler, this,
                    std::placeholders::_1),
          yasmin::Outcomes{},
          std::bind(&GenerateChatCompletionsState::result_handler, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&GenerateChatCompletionsState::feedback_handler, this,
                    std::placeholders::_1, std::placeholders::_2)),
      pipeline_(std::move(pipeline)), config_(config),
      history_(std::move(history)), memory_(std::move(memory)) {}

void GenerateChatCompletionsState::request_cancel() { this->cancel_state(); }

std::string GenerateChatCompletionsState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
  {
    std::lock_guard<std::mutex> lock(this->buffer_mutex_);
    this->accumulator_.clear();
  }

  std::string outcome;
  try {
    outcome = yasmin_ros::ActionState<
        llama_msgs::action::GenerateChatCompletions>::execute(blackboard);
  } catch (...) {
    // Never leave the SPEAK side blocked when generation dies unexpectedly.
    this->pipeline_->close(yasmin_ros::basic_outcomes::ABORT);
    throw;
  }
  this->pipeline_->close(outcome);
  return outcome;
}

std::string GenerateChatCompletionsState::build_system_prompt(
    const std::string &user_text) {
  std::string prompt = this->config_.system_prompt;
  if (!this->memory_ || !this->memory_->enabled()) {
    return prompt;
  }

  const std::vector<std::string> memories = this->memory_->retrieve(user_text);
  if (memories.empty()) {
    return prompt;
  }

  if (!prompt.empty()) {
    prompt += "\n\n";
  }
  prompt += "Things you remember from previous conversations. Use them only "
            "if they are relevant to the current request:";
  for (const std::string &memory : memories) {
    prompt += "\n- " + memory;
  }
  return prompt;
}

llama_msgs::action::GenerateChatCompletions::Goal
GenerateChatCompletionsState::create_goal_handler(
    yasmin::Blackboard::SharedPtr blackboard) {
  llama_msgs::action::GenerateChatCompletions::Goal goal;

  this->current_user_text_ = blackboard->get<std::string>("stt");

  const std::string system_prompt =
      this->build_system_prompt(this->current_user_text_);
  if (!system_prompt.empty()) {
    llama_msgs::msg::ChatMessage system_message;
    system_message.role = "system";
    system_message.content = system_prompt;
    goal.messages.push_back(system_message);
  }

  if (this->history_) {
    this->history_->add_user_message(this->current_user_text_);
    for (const llama_msgs::msg::ChatMessage &message :
         this->history_->messages(this->config_.history_max_turns)) {
      goal.messages.push_back(message);
    }
  } else {
    llama_msgs::msg::ChatMessage user_message;
    user_message.role = "user";
    user_message.content = this->current_user_text_;
    goal.messages.push_back(user_message);
  }

  goal.add_generation_prompt = true;
  goal.use_jinja = true;
  goal.stream = true;
  goal.sampling_config.temp = static_cast<float>(this->config_.temperature);
  return goal;
}

std::string GenerateChatCompletionsState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    llama_msgs::action::GenerateChatCompletions::Result::SharedPtr result) {
  (void)blackboard;
  const std::string content =
      result->choices.empty() ? "" : result->choices[0].message.content;
  const std::string spoken = text::SpeechSanitizer::sanitize(content);

  RCLCPP_INFO(this->node_->get_logger(), "%s", content.c_str());
  RCLCPP_INFO(this->node_->get_logger(), "Total tokens: %d",
              result->usage.total_tokens);

  if (this->history_ && !spoken.empty()) {
    this->history_->add_assistant_message(spoken);
  }
  if (this->memory_ && !spoken.empty()) {
    this->memory_->remember_turn(this->current_user_text_, spoken);
  }

  std::lock_guard<std::mutex> lock(this->buffer_mutex_);
  auto leftover = this->accumulator_.flush();
  if (leftover.has_value()) {
    const std::string cleaned = text::SpeechSanitizer::sanitize(*leftover);
    if (!cleaned.empty()) {
      this->pipeline_->push(cleaned);
    }
  }
  return yasmin_ros::basic_outcomes::SUCCEED;
}

void GenerateChatCompletionsState::feedback_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    std::shared_ptr<const llama_msgs::action::GenerateChatCompletions::Feedback>
        feedback) {
  (void)blackboard;
  std::lock_guard<std::mutex> lock(this->buffer_mutex_);
  for (const auto &choice : feedback->choices) {
    if (choice.delta.content.empty()) {
      continue;
    }
    const std::vector<std::string> sentences =
        this->accumulator_.add(choice.delta.content);
    for (const std::string &sentence : sentences) {
      const std::string cleaned = text::SpeechSanitizer::sanitize(sentence);
      if (!cleaned.empty()) {
        this->pipeline_->push(cleaned);
      }
    }
  }
}

} // namespace chatbot_ros
