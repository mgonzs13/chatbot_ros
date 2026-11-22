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

#include "chatbot_ros/states/listening_fsm.hpp"

#include <memory>

#include "chatbot_ros/states/check_stt_state.hpp"
#include "chatbot_ros/states/listen_state.hpp"
#include "chatbot_ros/states/speak_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace chatbot_ros {

ListenFSM::ListenFSM(const ChatbotConfig &config)
    : yasmin::StateMachine({yasmin_ros::basic_outcomes::SUCCEED,
                            yasmin_ros::basic_outcomes::CANCEL,
                            yasmin_ros::basic_outcomes::ABORT}) {

  this->add_state(
      "LISTENING", std::make_shared<ListenState>(config),
      {{yasmin_ros::basic_outcomes::SUCCEED, "CHECKING_STT"},
       {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
       {yasmin_ros::basic_outcomes::CANCEL,
        yasmin_ros::basic_outcomes::CANCEL}});

  this->add_state("CHECKING_STT",
                  std::make_shared<CheckSttState>(config.retry_messages),
                  {{yasmin_ros::basic_outcomes::SUCCEED,
                    yasmin_ros::basic_outcomes::SUCCEED},
                   {yasmin_ros::basic_outcomes::ABORT, "COMPLAINING"}});

  this->add_state(
      "COMPLAINING", std::make_shared<SpeakState>(),
      {{yasmin_ros::basic_outcomes::SUCCEED, "LISTENING"},
       {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
       {yasmin_ros::basic_outcomes::CANCEL,
        yasmin_ros::basic_outcomes::CANCEL}});
}

} // namespace chatbot_ros
