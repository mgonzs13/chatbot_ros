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

#include "chatbot_ros/states/check_stt_state.hpp"

#include "yasmin_ros/basic_outcomes.hpp"

namespace chatbot_ros {

CheckSttState::CheckSttState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT}) {}

std::string CheckSttState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  std::string transcript = "";
  if (blackboard->contains("stt")) {
    transcript = blackboard->get<std::string>("stt");
  }
  if (!transcript.empty()) {
    return yasmin_ros::basic_outcomes::SUCCEED;
  }
  blackboard->set<std::string>("tts",
                               "I didn't understand youd. Can you repeat?");
  return yasmin_ros::basic_outcomes::ABORT;
}

} // namespace chatbot_ros
