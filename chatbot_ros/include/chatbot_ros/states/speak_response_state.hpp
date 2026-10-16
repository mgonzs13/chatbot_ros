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

#ifndef CHATBOT_ROS__STATES__SPEAK_RESPONSE_STATE_HPP_
#define CHATBOT_ROS__STATES__SPEAK_RESPONSE_STATE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "audio_common_msgs/action/tts.hpp"
#include "chatbot_ros/states/speech_pipeline.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace chatbot_ros {

class SpeakResponseState : public yasmin::State {
public:
  explicit SpeakResponseState(SpeechPipeline::SharedPtr pipeline);

  void cancel_state() override;

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  bool speak(const std::string &sentence);

  /// Cancels the in-flight TTS goal (if any) and clears the handle.
  /// Safe to call from any thread and multiple times; never throws.
  void cancel_current_goal();

  SpeechPipeline::SharedPtr pipeline_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<audio_common_msgs::action::TTS>::SharedPtr tts_client_;
  std::mutex goal_mutex_;
  std::shared_ptr<
      rclcpp_action::ClientGoalHandle<audio_common_msgs::action::TTS>>
      current_goal_;
  std::atomic<bool> canceled_{false};
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__SPEAK_RESPONSE_STATE_HPP_
