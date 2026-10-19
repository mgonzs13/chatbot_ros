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

#include "chatbot_ros/states/listen_state.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "yasmin_ros/basic_outcomes.hpp"

using namespace std::chrono_literals;

namespace chatbot_ros {

namespace {
/// Number of extra attempts when the STT server rejects a goal because it
/// is still finishing the cancellation of the previous one.
constexpr int kMaxRejectedRetries = 3;
constexpr auto kRejectedRetryDelay = 150ms;
} // namespace

ListenState::ListenState(const ChatbotConfig &config)
    : yasmin_ros::ActionState<whisper_msgs::action::STT>(
          "/whisper/listen",
          std::bind(&ListenState::create_goal_handler, this,
                    std::placeholders::_1),
          yasmin::Outcomes{},
          std::bind(&ListenState::result_handler, this, std::placeholders::_1,
                    std::placeholders::_2)),
      config_(config), filter_(config.stt_ignored_phrases) {}

std::string ListenState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  for (int attempt = 0; attempt <= kMaxRejectedRetries; attempt++) {
    const std::string outcome =
        yasmin_ros::ActionState<whisper_msgs::action::STT>::execute(blackboard);
    if (outcome != yasmin_ros::basic_outcomes::ABORT || this->is_canceled()) {
      return outcome;
    }
    // The whisper server rejects new goals while it is still finishing the
    // cancellation of the previous one. Retry shortly before giving up.
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "Whisper rejected the goal, retrying (%d/%d)", attempt + 1,
                 kMaxRejectedRetries);
    std::this_thread::sleep_for(kRejectedRetryDelay);
  }
  return yasmin_ros::basic_outcomes::ABORT;
}

whisper_msgs::action::STT::Goal
ListenState::create_goal_handler(yasmin::Blackboard::SharedPtr blackboard) {
  (void)blackboard;
  whisper_msgs::action::STT::Goal goal;
  goal.prompt = this->config_.stt_prompt;
  return goal;
}

std::string ListenState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    whisper_msgs::action::STT::Result::SharedPtr result) {
  const std::string transcript =
      this->filter_.clean_or_empty(result->transcription.text);
  if (transcript != result->transcription.text) {
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "Filtered transcription: '%s' -> '%s'",
                 result->transcription.text.c_str(), transcript.c_str());
  }
  blackboard->set<std::string>("stt", transcript);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

} // namespace chatbot_ros
