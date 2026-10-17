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

#include "chatbot_ros/states/interrupt_listening_state.hpp"

#include <chrono>
#include <functional>
#include <thread>
#include <utility>

#include "yasmin_ros/basic_outcomes.hpp"

using namespace std::chrono_literals;

namespace chatbot_ros {

namespace {
/// Maximum consecutive STT failures before giving up for this cycle.
constexpr int kMaxFailures = 3;
constexpr auto kRetryDelay = 100ms;
} // namespace

InterruptListeningState::InterruptListeningState(
    const ChatbotConfig &config, EchoGuard::SharedPtr echo_guard)
    : yasmin_ros::ActionState<whisper_msgs::action::STT>(
          "/whisper/listen",
          std::bind(&InterruptListeningState::create_goal_handler, this,
                    std::placeholders::_1),
          yasmin::Outcomes{},
          std::bind(&InterruptListeningState::result_handler, this,
                    std::placeholders::_1, std::placeholders::_2)),
      config_(config), echo_guard_(std::move(echo_guard)),
      filter_(config.stt_ignored_phrases) {}

void InterruptListeningState::set_interrupt_callback(
    InterruptCallback callback) {
  this->interrupt_callback_ = std::move(callback);
}

void InterruptListeningState::stop() {
  this->stop_requested_.store(true);
  this->cancel_state();
}

void InterruptListeningState::reset_for_new_cycle() {
  this->stop_requested_.store(false);
}

whisper_msgs::action::STT::Goal InterruptListeningState::create_goal_handler(
    yasmin::Blackboard::SharedPtr blackboard) {
  (void)blackboard;
  whisper_msgs::action::STT::Goal goal;
  goal.prompt = this->config_.stt_prompt;
  return goal;
}

std::string InterruptListeningState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    whisper_msgs::action::STT::Result::SharedPtr result) {
  (void)blackboard;
  this->last_transcript_ = result->transcription.text;
  return yasmin_ros::basic_outcomes::SUCCEED;
}

std::string
InterruptListeningState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  int failures = 0;

  while (true) {
    if (this->stop_requested_.load() || this->is_canceled()) {
      return yasmin_ros::basic_outcomes::CANCEL;
    }

    const std::string outcome =
        yasmin_ros::ActionState<whisper_msgs::action::STT>::execute(blackboard);

    if (outcome != yasmin_ros::basic_outcomes::SUCCEED) {
      if (this->stop_requested_.load() || this->is_canceled()) {
        return yasmin_ros::basic_outcomes::CANCEL;
      }
      // Never abort the whole response because the listener failed.
      if (++failures >= kMaxFailures) {
        RCLCPP_WARN(this->node_->get_logger(),
                    "Interrupt listener failed %d times, giving up for this "
                    "response",
                    failures);
        return yasmin_ros::basic_outcomes::CANCEL;
      }
      std::this_thread::sleep_for(kRetryDelay);
      continue;
    }
    failures = 0;

    const std::string transcript = this->filter_.clean(this->last_transcript_);
    if (transcript.empty()) {
      RCLCPP_DEBUG(this->node_->get_logger(),
                   "Interrupt listener ignored non-speech transcript '%s'",
                   this->last_transcript_.c_str());
      continue;
    }

    if (this->echo_guard_ &&
        this->echo_guard_->is_echo(
            transcript, this->config_.interrupt_similarity_threshold)) {
      RCLCPP_DEBUG(this->node_->get_logger(),
                   "Ignoring echo of the robot's own voice: '%s'",
                   transcript.c_str());
      continue;
    }

    RCLCPP_INFO(this->node_->get_logger(), "User interrupted with: '%s'",
                transcript.c_str());
    blackboard->set<std::string>("stt", transcript);
    if (this->interrupt_callback_) {
      this->interrupt_callback_();
    }
    return yasmin_ros::basic_outcomes::SUCCEED;
  }
}

} // namespace chatbot_ros
