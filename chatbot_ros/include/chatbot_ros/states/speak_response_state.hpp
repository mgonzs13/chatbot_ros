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

#ifndef CHATBOT_ROS__STATES__SPEAK_RESPONSE_STATE_HPP_
#define CHATBOT_ROS__STATES__SPEAK_RESPONSE_STATE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "audio_common_msgs/action/tts.hpp"
#include "chatbot_ros/conversation_history.hpp"
#include "chatbot_ros/echo_guard.hpp"
#include "chatbot_ros/embedding_memory.hpp"
#include "chatbot_ros/states/speech_pipeline.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace chatbot_ros {

class SpeakResponseState : public yasmin::State {
public:
  using FinishedCallback = std::function<void()>;

  explicit SpeakResponseState(SpeechPipeline::SharedPtr pipeline,
                              EchoGuard::SharedPtr echo_guard = nullptr,
                              ConversationHistory::SharedPtr history = nullptr,
                              EmbeddingMemory::SharedPtr memory = nullptr);

  /// Callback invoked whenever the response cycle ends (success, abort or
  /// cancel). Used to stop the concurrent interrupt listener.
  void set_on_finished(FinishedCallback callback);

  /// Stops speaking immediately because the user interrupted.
  void interrupt();

  /// Prepares the state for a new response cycle.
  void reset_for_new_cycle();

  void cancel_state() override;

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  bool speak(const std::string &sentence);

  /// Cancels the in-flight TTS goal (if any) and clears the handle.
  /// Safe to call from any thread and multiple times; never throws.
  void cancel_current_goal();

  void notify_finished();

  SpeechPipeline::SharedPtr pipeline_;
  EchoGuard::SharedPtr echo_guard_;
  ConversationHistory::SharedPtr history_;
  EmbeddingMemory::SharedPtr memory_;
  FinishedCallback on_finished_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<audio_common_msgs::action::TTS>::SharedPtr tts_client_;
  std::mutex goal_mutex_;
  std::shared_ptr<
      rclcpp_action::ClientGoalHandle<audio_common_msgs::action::TTS>>
      current_goal_;
  std::atomic<bool> canceled_{false};
  std::atomic<bool> interrupted_{false};
  /// Sentences that were spoken successfully in this response cycle.
  std::string spoken_text_;
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__SPEAK_RESPONSE_STATE_HPP_
