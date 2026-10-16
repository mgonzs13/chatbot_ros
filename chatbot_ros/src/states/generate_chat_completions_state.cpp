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

#include "yasmin_ros/basic_outcomes.hpp"

namespace chatbot_ros {

GenerateChatCompletionsState::GenerateChatCompletionsState(
    SpeechPipeline::SharedPtr pipeline)
    : yasmin_ros::ActionState<llama_msgs::action::GenerateChatCompletions>(
          "/llama/generate_chat_completions",
          std::bind(&GenerateChatCompletionsState::create_goal_handler, this,
                    std::placeholders::_1),
          yasmin::Outcomes{},
          std::bind(&GenerateChatCompletionsState::result_handler, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&GenerateChatCompletionsState::feedback_handler, this,
                    std::placeholders::_1, std::placeholders::_2)),
      pipeline_(std::move(pipeline)) {}

std::string GenerateChatCompletionsState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
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

llama_msgs::action::GenerateChatCompletions::Goal
GenerateChatCompletionsState::create_goal_handler(
    yasmin::Blackboard::SharedPtr blackboard) {
  llama_msgs::action::GenerateChatCompletions::Goal goal;

  llama_msgs::msg::ChatMessage message;
  message.role = "user";
  message.content = blackboard->get<std::string>("stt");
  goal.messages.push_back(message);

  goal.add_generation_prompt = true;
  goal.use_jinja = true;
  goal.stream = true;
  goal.sampling_config.temp = 0.0F;
  return goal;
}

std::string GenerateChatCompletionsState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    llama_msgs::action::GenerateChatCompletions::Result::SharedPtr result) {
  (void)blackboard;
  const std::string content =
      result->choices.empty() ? "" : result->choices[0].message.content;
  RCLCPP_INFO(this->node_->get_logger(), "%s", content.c_str());
  RCLCPP_INFO(this->node_->get_logger(), "Total tokens: %d",
              result->usage.total_tokens);
  std::lock_guard<std::mutex> lock(this->buffer_mutex_);
  auto leftover = this->accumulator_.flush();
  if (leftover.has_value()) {
    this->pipeline_->push(*leftover);
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
      this->pipeline_->push(sentence);
    }
  }
}

} // namespace chatbot_ros
