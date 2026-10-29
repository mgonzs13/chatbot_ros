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

#include <algorithm>

#include "chatbot_ros/text/text_similarity.hpp"

namespace chatbot_ros {

void EchoGuard::add_spoken_sentence(const std::string &sentence) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->spoken_.push_back(sentence);
  while (this->spoken_.size() > kMaxSentences) {
    this->spoken_.pop_front();
  }
}

void EchoGuard::set_speaking(bool speaking) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->speaking_ = speaking;
}

void EchoGuard::clear() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->spoken_.clear();
}

bool EchoGuard::is_echo(const std::string &candidate, double threshold,
                        const std::vector<std::string> &always_words) const {
  const std::vector<std::string> tokens = text::tokenize(candidate);
  if (tokens.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(this->mutex_);
  if (this->spoken_.empty()) {
    return false;
  }

  const std::vector<std::string> current_tokens =
      text::tokenize(this->spoken_.back());

  // While the robot is speaking, a short fragment made only of words from
  // the current sentence is almost always its own voice. This catches
  // Whisper fragments such as "Thanks." or "No, no." from the sentence being
  // played, which are too short for the coverage test below.
  if (this->speaking_ && tokens.size() <= 3) {
    const bool all_in_current = std::all_of(
        tokens.begin(), tokens.end(),
        [&current_tokens](const std::string &token) {
          return std::find(current_tokens.begin(), current_tokens.end(),
                           token) != current_tokens.end();
        });
    if (all_in_current) {
      return true;
    }
  }

  // A full echo of the sentence currently being spoken. This is checked
  // before command words so a complete echo containing "stop" is still
  // filtered; a real command that is not an echo falls through.
  if (text::subsequence_similarity(this->spoken_.back(), candidate) >=
      threshold) {
    return true;
  }

  // A command word that is not an echo of the current sentence is a real
  // interruption, even if it happens to match an older buffered sentence.
  for (const std::string &token : tokens) {
    if (std::find(always_words.begin(), always_words.end(), token) !=
        always_words.end()) {
      return false;
    }
  }

  // Older sentences may still be buffered in the player. Compare against
  // them without letting them override a real command word.
  for (std::size_t i = 0; i + 1 < this->spoken_.size(); ++i) {
    // Measure how much of the spoken sentence the transcript covers, in
    // order. Comparing against the candidate length would classify any
    // short request contained in a long sentence as echo.
    if (text::subsequence_similarity(this->spoken_[i], candidate) >=
        threshold) {
      return true;
    }
  }
  return false;
}

} // namespace chatbot_ros
