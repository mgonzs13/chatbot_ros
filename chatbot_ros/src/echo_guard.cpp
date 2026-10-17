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

namespace {
/// Transcriptions with at most this many words are checked against the
/// always-interrupt command words.
constexpr std::size_t kAlwaysWordMaxTokens = 3;

/// Single words are only considered an echo when they are at least this
/// long, so short real utterances ("stop", "no") always interrupt.
constexpr std::size_t kMinEchoWordLength = 4;
} // namespace

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

bool EchoGuard::is_echo(const std::string &candidate, double threshold,
                        const std::vector<std::string> &always_words) const {
  const std::vector<std::string> tokens = text::tokenize(candidate);
  if (tokens.empty()) {
    return false;
  }

  // Short commands such as "stop" or "be quiet" always interrupt, even if
  // their words show up in the robot's own speech.
  if (tokens.size() <= kAlwaysWordMaxTokens) {
    for (const std::string &token : tokens) {
      if (std::find(always_words.begin(), always_words.end(), token) !=
          always_words.end()) {
        return false;
      }
    }
  }

  std::lock_guard<std::mutex> lock(this->mutex_);

  if (tokens.size() < 2) {
    // A single word is likely a fragment of the sentence that is being
    // spoken (e.g. Whisper only caught "Sure." from "Sure, I can do that").
    if (tokens[0].size() < kMinEchoWordLength) {
      return false;
    }
    for (const std::string &spoken : this->spoken_) {
      const std::vector<std::string> spoken_tokens = text::tokenize(spoken);
      if (std::find(spoken_tokens.begin(), spoken_tokens.end(), tokens[0]) !=
          spoken_tokens.end()) {
        return true;
      }
    }
    return false;
  }

  for (const std::string &spoken : this->spoken_) {
    // Order-preserving match: tolerates Whisper dropping words from the
    // robot's speech or hallucinating extra words at the end.
    if (text::subsequence_similarity(candidate, spoken) >= threshold) {
      return true;
    }
  }
  return false;
}

} // namespace chatbot_ros
