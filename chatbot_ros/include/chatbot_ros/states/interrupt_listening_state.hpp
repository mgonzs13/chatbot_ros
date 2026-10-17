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

#ifndef CHATBOT_ROS__STATES__INTERRUPT_LISTENING_STATE_HPP_
#define CHATBOT_ROS__STATES__INTERRUPT_LISTENING_STATE_HPP_

#include <atomic>
#include <functional>
#include <string>

#include "chatbot_ros/chatbot_config.hpp"
#include "chatbot_ros/echo_guard.hpp"
#include "chatbot_ros/text/transcript_filter.hpp"
#include "whisper_msgs/action/stt.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_ros/action_state.hpp"

namespace chatbot_ros {

/// Listens for user speech while the chatbot is generating and speaking a
/// response. When a real utterance is recognized (not noise and not an
/// echo of the robot's own voice) it is written to the `stt` blackboard
/// key and the interruption callback cancels the ongoing response.
class InterruptListeningState
    : public yasmin_ros::ActionState<whisper_msgs::action::STT> {
public:
  using InterruptCallback = std::function<void()>;

  explicit InterruptListeningState(
      const ChatbotConfig &config = ChatbotConfig(),
      EchoGuard::SharedPtr echo_guard = nullptr);

  /// Invoked when the user interrupts the response.
  void set_interrupt_callback(InterruptCallback callback);

  /// Stops listening, e.g. because the robot finished speaking normally.
  void stop();

  /// Prepares the state for a new response cycle.
  void reset_for_new_cycle();

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  whisper_msgs::action::STT::Goal
  create_goal_handler(yasmin::Blackboard::SharedPtr blackboard);

  std::string
  result_handler(yasmin::Blackboard::SharedPtr blackboard,
                 whisper_msgs::action::STT::Result::SharedPtr result);

  ChatbotConfig config_;
  EchoGuard::SharedPtr echo_guard_;
  text::TranscriptFilter filter_;
  InterruptCallback interrupt_callback_;
  std::atomic<bool> stop_requested_{false};
  std::string last_transcript_;
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__INTERRUPT_LISTENING_STATE_HPP_
