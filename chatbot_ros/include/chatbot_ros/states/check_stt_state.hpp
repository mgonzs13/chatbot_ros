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
