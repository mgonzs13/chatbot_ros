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
