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

#include <functional>
#include <string>

#include "yasmin_ros/basic_outcomes.hpp"

namespace chatbot_ros {

ListenState::ListenState()
    : yasmin_ros::ActionState<whisper_msgs::action::STT>(
          "/whisper/listen",
          std::bind(&ListenState::create_goal_handler, this,
                    std::placeholders::_1),
          yasmin::Outcomes{},
          std::bind(&ListenState::result_handler, this, std::placeholders::_1,
                    std::placeholders::_2)) {}

whisper_msgs::action::STT::Goal
ListenState::create_goal_handler(yasmin::Blackboard::SharedPtr blackboard) {
  (void)blackboard;
  return whisper_msgs::action::STT::Goal();
}

std::string ListenState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    whisper_msgs::action::STT::Result::SharedPtr result) {
  blackboard->set<std::string>("stt", result->transcription.text);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

} // namespace chatbot_ros
