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

SpeakResponseState::SpeakResponseState(SpeechPipeline::SharedPtr pipeline,
                                       EchoGuard::SharedPtr echo_guard,
                                       ConversationHistory::SharedPtr history,
                                       EmbeddingMemory::SharedPtr memory)
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT,
                     yasmin_ros::basic_outcomes::CANCEL}),
      pipeline_(std::move(pipeline)), echo_guard_(std::move(echo_guard)),
      history_(std::move(history)), memory_(std::move(memory)),
      node_(yasmin_ros::YasminNode::get_instance()) {
  this->tts_client_ =
      rclcpp_action::create_client<audio_common_msgs::action::TTS>(this->node_,
                                                                   "/say");
}

void SpeakResponseState::set_on_finished(FinishedCallback callback) {
  this->on_finished_ = std::move(callback);
}

void SpeakResponseState::notify_finished() {
  if (this->on_finished_) {
    this->on_finished_();
  }
}

void SpeakResponseState::interrupt() {
  this->interrupted_.store(true);
  this->pipeline_->cancel();
  this->cancel_current_goal();
}

void SpeakResponseState::reset_for_new_cycle() {
  this->interrupted_.store(false);
  this->spoken_text_.clear();
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
  this->spoken_text_.clear();

  // Always wake up the interrupt listener when this state ends, whatever
  // the outcome is; otherwise the responding concurrence would keep
  // listening forever after the robot finished speaking.
  struct FinishGuard {
    explicit FinishGuard(SpeakResponseState *state) : state(state) {}
    ~FinishGuard() { this->state->notify_finished(); }
    SpeakResponseState *state;
  } finish_guard(this);

  std::string sentence;
  while (this->pipeline_->pop(sentence)) {
    if (!this->speak(sentence)) {
      if (this->interrupted_.load() || this->canceled_.load() ||
          this->is_canceled()) {
        return yasmin_ros::basic_outcomes::CANCEL;
      }
      return yasmin_ros::basic_outcomes::ABORT;
    }
    if (!this->spoken_text_.empty()) {
      this->spoken_text_ += " ";
    }
    this->spoken_text_ += sentence;
  }

  if (this->interrupted_.load()) {
    return yasmin_ros::basic_outcomes::CANCEL;
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

  // Only commit the turn after the whole response was spoken. If the user
  // interrupted, the assistant message was never delivered and must not be
  // sent back to the LLM or stored in long-term memory.
  if (!this->spoken_text_.empty()) {
    if (this->history_) {
      this->history_->add_assistant_message(this->spoken_text_);
    }
    if (this->memory_) {
      std::string user_text;
      if (blackboard->contains("stt")) {
        user_text = blackboard->get<std::string>("stt");
      }
      this->memory_->remember_turn(user_text, this->spoken_text_);
    }
  }
  return yasmin_ros::basic_outcomes::SUCCEED;
}

bool SpeakResponseState::speak(const std::string &sentence) {
  if (sentence.empty()) {
    return true;
  }

  // Register the sentence before it plays so the interrupt listener can
  // recognize the echo of the robot's own voice.
  if (this->echo_guard_) {
    this->echo_guard_->add_spoken_sentence(sentence);
  }

  // Mark the current sentence as playing for the whole TTS goal. The guard
  // clears the flag on every return, including cancellation.
  struct SpeakingGuard {
    explicit SpeakingGuard(const EchoGuard::SharedPtr &guard) : guard(guard) {
      if (this->guard) {
        this->guard->set_speaking(true);
      }
    }
    ~SpeakingGuard() {
      if (this->guard) {
        this->guard->set_speaking(false);
      }
    }
    EchoGuard::SharedPtr guard;
  } speaking_guard(this->echo_guard_);

  while (!this->tts_client_->wait_for_action_server(1s)) {
    if (!rclcpp::ok() || this->interrupted_.load() || this->canceled_.load() ||
        this->is_canceled()) {
      return false;
    }
  }

  audio_common_msgs::action::TTS::Goal goal;
  goal.text = sentence;

  auto goal_handle_future = this->tts_client_->async_send_goal(goal);
  while (goal_handle_future.wait_for(100ms) != std::future_status::ready) {
    if (!rclcpp::ok() || this->interrupted_.load() || this->canceled_.load() ||
        this->is_canceled()) {
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
  if (this->interrupted_.load() || this->canceled_.load() ||
      this->is_canceled()) {
    this->cancel_current_goal();
    return false;
  }

  auto result_future = this->tts_client_->async_get_result(goal_handle);
  while (result_future.wait_for(100ms) != std::future_status::ready) {
    if (!rclcpp::ok() || this->interrupted_.load() || this->canceled_.load() ||
        this->is_canceled()) {
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
