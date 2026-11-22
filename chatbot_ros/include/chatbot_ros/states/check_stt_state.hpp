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

#ifndef CHATBOT_ROS__STATES__CHECK_STT_STATE_HPP_
#define CHATBOT_ROS__STATES__CHECK_STT_STATE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace chatbot_ros {

class CheckSttState : public yasmin::State {
public:
  /// `retry_messages` are spoken in round-robin order whenever the
  /// transcription is empty; an empty list uses the defaults.
  explicit CheckSttState(std::vector<std::string> retry_messages = {});

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  std::vector<std::string> retry_messages_;
  std::size_t retry_index_{0};
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__CHECK_STT_STATE_HPP_
