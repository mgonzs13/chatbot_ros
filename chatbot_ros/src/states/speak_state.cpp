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

#include "chatbot_ros/states/speak_state.hpp"

#include <functional>
#include <string>

namespace chatbot_ros {

SpeakState::SpeakState()
    : yasmin_ros::ActionState<audio_common_msgs::action::TTS>(
          "/say", std::bind(&SpeakState::create_goal_handler, this,
                            std::placeholders::_1)) {}

audio_common_msgs::action::TTS::Goal
SpeakState::create_goal_handler(yasmin::Blackboard::SharedPtr blackboard) {
  audio_common_msgs::action::TTS::Goal goal;
  goal.text = blackboard->get<std::string>("tts");
  return goal;
}

} // namespace chatbot_ros
