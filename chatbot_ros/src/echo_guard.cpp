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

#include "chatbot_ros/echo_guard.hpp"

#include "chatbot_ros/text/text_similarity.hpp"

namespace chatbot_ros {

void EchoGuard::add_spoken_sentence(const std::string &sentence) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->spoken_.push_back(sentence);
  while (this->spoken_.size() > kMaxSentences) {
    this->spoken_.pop_front();
  }
}

void EchoGuard::clear() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->spoken_.clear();
}

bool EchoGuard::is_echo(const std::string &candidate, double threshold) const {
  const std::vector<std::string> tokens = text::tokenize(candidate);
  if (tokens.size() < 2) {
    // Single words are more likely a command ("stop", "wait") than echo.
    return false;
  }

  std::lock_guard<std::mutex> lock(this->mutex_);
  for (const std::string &spoken : this->spoken_) {
    if (text::similarity(candidate, spoken) >= threshold) {
      return true;
    }
  }
  return false;
}

} // namespace chatbot_ros
