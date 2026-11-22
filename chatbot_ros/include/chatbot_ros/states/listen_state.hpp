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

#ifndef CHATBOT_ROS__STATES__LISTEN_STATE_HPP_
#define CHATBOT_ROS__STATES__LISTEN_STATE_HPP_

#include <string>

#include "chatbot_ros/chatbot_config.hpp"
#include "chatbot_ros/text/transcript_filter.hpp"
#include "whisper_msgs/action/stt.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_ros/action_state.hpp"

namespace chatbot_ros {

class ListenState : public yasmin_ros::ActionState<whisper_msgs::action::STT> {
public:
  explicit ListenState(const ChatbotConfig &config = ChatbotConfig());

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  whisper_msgs::action::STT::Goal
  create_goal_handler(yasmin::Blackboard::SharedPtr blackboard);

  std::string
  result_handler(yasmin::Blackboard::SharedPtr blackboard,
                 whisper_msgs::action::STT::Result::SharedPtr result);

  ChatbotConfig config_;
  text::TranscriptFilter filter_;
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__LISTEN_STATE_HPP_
