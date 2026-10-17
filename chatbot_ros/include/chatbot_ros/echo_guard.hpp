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

#ifndef CHATBOT_ROS__ECHO_GUARD_HPP_
#define CHATBOT_ROS__ECHO_GUARD_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace chatbot_ros {

/// Keeps the sentences the robot is speaking so transcriptions of the
/// robot's own voice (picked up by the microphone and transcribed by
/// Whisper) can be told apart from real user interruptions.
///
/// A transcription is considered an echo when it mostly repeats, in order,
/// the words of a recently spoken sentence. Missing words, extra
/// hallucinated words and partial fragments are tolerated.
class EchoGuard {
public:
  using SharedPtr = std::shared_ptr<EchoGuard>;

  /// Registers a sentence before it starts playing.
  void add_spoken_sentence(const std::string &sentence);

  /// Empties the history, e.g. at the start of a new response cycle.
  void clear();

  /// True when the candidate transcription looks like an echo of one of
  /// the recently spoken sentences.
  ///
  /// `always_words` are command words (e.g. "stop") that always count as a
  /// real interruption when they show up in a short transcription.
  bool is_echo(const std::string &candidate, double threshold,
               const std::vector<std::string> &always_words = {}) const;

private:
  static constexpr std::size_t kMaxSentences = 8;

  mutable std::mutex mutex_;
  std::deque<std::string> spoken_;
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__ECHO_GUARD_HPP_
