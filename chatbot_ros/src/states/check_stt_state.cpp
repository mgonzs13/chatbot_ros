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

#include "chatbot_ros/states/check_stt_state.hpp"

#include <utility>

#include "chatbot_ros/chatbot_config.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace chatbot_ros {

CheckSttState::CheckSttState(std::vector<std::string> retry_messages)
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT}) {
  if (retry_messages.empty()) {
    retry_messages = default_retry_messages();
  }
  this->retry_messages_ = std::move(retry_messages);
}

std::string CheckSttState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  std::string transcript = "";
  if (blackboard->contains("stt")) {
    transcript = blackboard->get<std::string>("stt");
  }
  if (!transcript.empty()) {
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  const std::string &message =
      this->retry_messages_[this->retry_index_ % this->retry_messages_.size()];
  this->retry_index_++;
  blackboard->set<std::string>("tts", message);
  return yasmin_ros::basic_outcomes::ABORT;
}

} // namespace chatbot_ros
