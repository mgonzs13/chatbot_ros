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
    // cancellation of the previous one, which happens right after the user
    // interrupted the robot. Retry shortly before giving up.
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
