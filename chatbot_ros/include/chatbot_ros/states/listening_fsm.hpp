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

#ifndef CHATBOT_ROS__STATES__LISTENING_FSM_HPP_
#define CHATBOT_ROS__STATES__LISTENING_FSM_HPP_

#include "chatbot_ros/chatbot_config.hpp"
#include "yasmin/state_machine.hpp"

namespace chatbot_ros {

class ListenFSM : public yasmin::StateMachine {
public:
  explicit ListenFSM(const ChatbotConfig &config = ChatbotConfig());
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__LISTENING_FSM_HPP_
