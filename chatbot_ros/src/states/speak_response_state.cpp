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

#include "chatbot_ros/states/speak_response_state.hpp"

#include <chrono>
#include <exception>
#include <future>
#include <utility>

#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace std::chrono_literals;

namespace chatbot_ros {

SpeakResponseState::SpeakResponseState(SpeechPipeline::SharedPtr pipeline)
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT,
                     yasmin_ros::basic_outcomes::CANCEL}),
      pipeline_(std::move(pipeline)),
      node_(yasmin_ros::YasminNode::get_instance()) {
  this->tts_client_ =
      rclcpp_action::create_client<audio_common_msgs::action::TTS>(this->node_,
                                                                   "/say");
}

void SpeakResponseState::cancel_state() {
  this->canceled_.store(true);
  this->pipeline_->cancel();
  this->cancel_current_goal();
  yasmin::State::cancel_state();
}

void SpeakResponseState::cancel_current_goal() {
  std::shared_ptr<
      rclcpp_action::ClientGoalHandle<audio_common_msgs::action::TTS>>
      goal;
  {
    std::lock_guard<std::mutex> lock(this->goal_mutex_);
    goal = this->current_goal_;
    this->current_goal_.reset();
  }
  if (!goal) {
    return;
  }
  try {
    this->tts_client_->async_cancel_goal(goal);
  } catch (const std::exception &e) {
    RCLCPP_DEBUG(this->node_->get_logger(), "Could not cancel TTS goal: %s",
                 e.what());
  }
}

std::string
SpeakResponseState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  (void)blackboard;

  std::string sentence;
  while (this->pipeline_->pop(sentence)) {
    if (!this->speak(sentence)) {
      if (this->canceled_.load() || this->is_canceled()) {
        return yasmin_ros::basic_outcomes::CANCEL;
      }
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  if (this->canceled_.load() || this->is_canceled()) {
    return yasmin_ros::basic_outcomes::CANCEL;
  }

  const std::string generation_outcome =
      this->pipeline_->get_generation_outcome();
  if (generation_outcome == yasmin_ros::basic_outcomes::CANCEL) {
    return yasmin_ros::basic_outcomes::CANCEL;
  }
  if (generation_outcome != yasmin_ros::basic_outcomes::SUCCEED) {
    return yasmin_ros::basic_outcomes::ABORT;
  }
  return yasmin_ros::basic_outcomes::SUCCEED;
}

bool SpeakResponseState::speak(const std::string &sentence) {
  if (sentence.empty()) {
    return true;
  }

  while (!this->tts_client_->wait_for_action_server(1s)) {
    if (!rclcpp::ok() || this->canceled_.load() || this->is_canceled()) {
      return false;
    }
  }

  audio_common_msgs::action::TTS::Goal goal;
  goal.text = sentence;

  auto goal_handle_future = this->tts_client_->async_send_goal(goal);
  while (goal_handle_future.wait_for(100ms) != std::future_status::ready) {
    if (!rclcpp::ok() || this->canceled_.load() || this->is_canceled()) {
      // Wait briefly for a possibly accepted goal so it can be canceled.
      if (goal_handle_future.wait_for(1s) == std::future_status::ready) {
        std::shared_ptr<
            rclcpp_action::ClientGoalHandle<audio_common_msgs::action::TTS>>
            handle = goal_handle_future.get();
        if (handle) {
          std::lock_guard<std::mutex> lock(this->goal_mutex_);
          this->current_goal_ = handle;
        }
        this->cancel_current_goal();
      }
      return false;
    }
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->node_->get_logger(), "TTS goal was rejected: '%s'",
                 sentence.c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(this->goal_mutex_);
    this->current_goal_ = goal_handle;
  }
  if (this->canceled_.load() || this->is_canceled()) {
    this->cancel_current_goal();
    return false;
  }

  auto result_future = this->tts_client_->async_get_result(goal_handle);
  while (result_future.wait_for(100ms) != std::future_status::ready) {
    if (!rclcpp::ok() || this->canceled_.load() || this->is_canceled()) {
      this->cancel_current_goal();
      return false;
    }
  }
  auto wrapped_result = result_future.get();
  {
    std::lock_guard<std::mutex> lock(this->goal_mutex_);
    this->current_goal_.reset();
  }

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_DEBUG(this->node_->get_logger(), "TTS goal was canceled: '%s'",
                   sentence.c_str());
    } else {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "TTS goal failed with code %d: '%s'",
                   static_cast<int>(wrapped_result.code), sentence.c_str());
    }
    return false;
  }
  return true;
}

} // namespace chatbot_ros
