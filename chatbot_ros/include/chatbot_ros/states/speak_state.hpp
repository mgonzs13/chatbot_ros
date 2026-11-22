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

#ifndef CHATBOT_ROS__STATES__SPEAK_STATE_HPP_
#define CHATBOT_ROS__STATES__SPEAK_STATE_HPP_

#include "audio_common_msgs/action/tts.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_ros/action_state.hpp"

namespace chatbot_ros {

class SpeakState
    : public yasmin_ros::ActionState<audio_common_msgs::action::TTS> {
public:
  SpeakState();

private:
  audio_common_msgs::action::TTS::Goal
  create_goal_handler(yasmin::Blackboard::SharedPtr blackboard);
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__SPEAK_STATE_HPP_
