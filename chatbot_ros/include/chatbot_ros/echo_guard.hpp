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
/// A transcription is considered an echo when it covers most of a recently
/// spoken sentence, in order. While the robot is speaking, short fragments
/// made only of words from the current sentence are also treated as echo.
/// Missing words and extra hallucinated words are tolerated; a request that
/// uses words outside the current sentence is not treated as echo.
class EchoGuard {
public:
  using SharedPtr = std::shared_ptr<EchoGuard>;

  /// Registers a sentence before it starts playing.
  void add_spoken_sentence(const std::string &sentence);

  /// Marks whether the robot is currently speaking. Short fragments of the
  /// current sentence are only suppressed while speaking.
  void set_speaking(bool speaking);

  /// Empties the history, e.g. at the start of a new response cycle.
  void clear();

  /// True when the candidate transcription looks like an echo of one of
  /// the recently spoken sentences.
  ///
  /// `always_words` are command words (e.g. "stop") that count as a real
  /// interruption unless they are part of the sentence being spoken.
  bool is_echo(const std::string &candidate, double threshold,
               const std::vector<std::string> &always_words = {}) const;

private:
  static constexpr std::size_t kMaxSentences = 8;

  mutable std::mutex mutex_;
  std::deque<std::string> spoken_;
  bool speaking_{false};
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__ECHO_GUARD_HPP_
